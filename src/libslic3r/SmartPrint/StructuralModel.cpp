#include "StructuralModel.hpp"

#include <Eigen/Dense>
#include <Eigen/Sparse>
#include <Eigen/SparseCholesky>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <limits>
#include <numeric>
#include <atomic>
#include <thread>
#include <unordered_map>
#include <utility>

namespace Slic3r { namespace SmartPrint {

namespace {

using Mat24     = Eigen::Matrix<double, 24, 24>;
using Mat6x24   = Eigen::Matrix<double, 6, 24>;
using Mat6      = Eigen::Matrix<double, 6, 6>;
using Vec6      = Eigen::Matrix<double, 6, 1>;
using Vec24     = Eigen::Matrix<double, 24, 1>;
using SpMat     = Eigen::SparseMatrix<double, Eigen::ColMajor, int>;
using Cholesky  = Eigen::SimplicialLDLT<SpMat, Eigen::Lower, Eigen::AMDOrdering<int>>;

constexpr double kInf          = 1e20;
constexpr int    kSubRays      = 3;    // sub-rays per voxel column and axis, for fill fractions
constexpr double kMaxSafety    = 100.0;
constexpr double kShearFactor  = 0.8;  // interlayer shear strength relative to interlayer tensile strength
constexpr double kStandardG    = 9.80665;
constexpr double kPi           = 3.14159265358979323846;

// Local node order of a brick: bit 0 = +x, bit 1 = +y, bit 2 = +z.
constexpr int kCorner[8][3] = {{0, 0, 0}, {1, 0, 0}, {0, 1, 0}, {1, 1, 0}, {0, 0, 1}, {1, 0, 1}, {0, 1, 1}, {1, 1, 1}};

Mat6 isotropic_d(double poisson)
{
    const double nu = poisson;
    const double c  = 1.0 / ((1.0 + nu) * (1.0 - 2.0 * nu));
    Mat6         d  = Mat6::Zero();
    for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 3; ++j)
            d(i, j) = c * (i == j ? 1.0 - nu : nu);
    for (int i = 3; i < 6; ++i)
        d(i, i) = c * (1.0 - 2.0 * nu) / 2.0;
    return d;
}

// Strain-displacement rows for one shape function with gradient g. Strain order: xx, yy, zz, xy, yz, zx.
template<class Mat> void put_b(Mat& b, int col, const Eigen::Vector3d& g)
{
    b(0, col + 0) = g.x();
    b(1, col + 1) = g.y();
    b(2, col + 2) = g.z();
    b(3, col + 0) = g.y();
    b(3, col + 1) = g.x();
    b(4, col + 1) = g.z();
    b(4, col + 2) = g.y();
    b(5, col + 0) = g.z();
    b(5, col + 2) = g.x();
}

// Brick of unit size and unit Young's modulus with Wilson's incompatible modes, statically condensed.
// The cube has a constant Jacobian, so the element passes the patch test; it removes the shear locking
// that makes plain 8-node bricks far too stiff in bending.
void unit_brick(double poisson, Mat24& k, Mat6x24& b_center, Mat6& d)
{
    d = isotropic_d(poisson);
    Eigen::Matrix<double, 24, 24> kuu = Eigen::Matrix<double, 24, 24>::Zero();
    Eigen::Matrix<double, 24, 9>  kua = Eigen::Matrix<double, 24, 9>::Zero();
    Eigen::Matrix<double, 9, 9>   kaa = Eigen::Matrix<double, 9, 9>::Zero();
    const double                  gp  = 1.0 / std::sqrt(3.0);
    const double                  dxi = 2.0; // d(xi)/dx for a unit cube
    for (int gi = 0; gi < 8; ++gi) {
        const double xi[3] = {kCorner[gi][0] ? gp : -gp, kCorner[gi][1] ? gp : -gp, kCorner[gi][2] ? gp : -gp};
        Mat6x24                     b  = Mat6x24::Zero();
        Eigen::Matrix<double, 6, 9> ba = Eigen::Matrix<double, 6, 9>::Zero();
        for (int a = 0; a < 8; ++a) {
            const double s[3] = {kCorner[a][0] ? 1.0 : -1.0, kCorner[a][1] ? 1.0 : -1.0, kCorner[a][2] ? 1.0 : -1.0};
            Eigen::Vector3d g;
            g.x() = 0.125 * s[0] * (1 + s[1] * xi[1]) * (1 + s[2] * xi[2]) * dxi;
            g.y() = 0.125 * s[1] * (1 + s[0] * xi[0]) * (1 + s[2] * xi[2]) * dxi;
            g.z() = 0.125 * s[2] * (1 + s[0] * xi[0]) * (1 + s[1] * xi[1]) * dxi;
            put_b(b, 3 * a, g);
        }
        for (int mode = 0; mode < 3; ++mode) {
            // Bubble 1 - xi_mode^2: gradient only along its own axis.
            Eigen::Vector3d g = Eigen::Vector3d::Zero();
            g[mode]           = -2.0 * xi[mode] * dxi;
            put_b(ba, 3 * mode, g);
        }
        const double w = 1.0 / 8.0; // det J of a unit cube mapped from [-1,1]^3, Gauss weights are 1
        kuu += w * b.transpose() * d * b;
        kua += w * b.transpose() * d * ba;
        kaa += w * ba.transpose() * d * ba;
    }
    k = kuu - kua * kaa.ldlt().solve(kua.transpose());
    k = 0.5 * (k + k.transpose());

    b_center.setZero();
    for (int a = 0; a < 8; ++a) {
        const double    s[3] = {kCorner[a][0] ? 1.0 : -1.0, kCorner[a][1] ? 1.0 : -1.0, kCorner[a][2] ? 1.0 : -1.0};
        Eigen::Vector3d g(0.125 * s[0] * dxi, 0.125 * s[1] * dxi, 0.125 * s[2] * dxi);
        put_b(b_center, 3 * a, g);
    }
}

// Felzenszwalb & Huttenlocher exact 1D squared distance transform.
void distance_transform_1d(const double* f, int n, double* out, std::vector<int>& v, std::vector<double>& z)
{
    v.resize(n);
    z.resize(n + 1);
    int k = 0;
    v[0]  = 0;
    z[0]  = -kInf;
    z[1]  = kInf;
    for (int q = 1; q < n; ++q) {
        double s = ((f[q] + double(q) * q) - (f[v[k]] + double(v[k]) * v[k])) / (2.0 * q - 2.0 * v[k]);
        while (s <= z[k]) {
            --k;
            s = ((f[q] + double(q) * q) - (f[v[k]] + double(v[k]) * v[k])) / (2.0 * q - 2.0 * v[k]);
        }
        ++k;
        v[k]     = q;
        z[k]     = s;
        z[k + 1] = kInf;
    }
    k = 0;
    for (int q = 0; q < n; ++q) {
        while (z[k + 1] < q)
            ++k;
        out[q] = double(q - v[k]) * (q - v[k]) + f[v[k]];
    }
}

double band_fraction(double distance, double thickness, double h)
{
    // Share of a voxel spanning [distance - h/2, distance + h/2] from the surface that lies in the solid band.
    return std::clamp((thickness - (distance - 0.5 * h)) / h, 0.0, 1.0);
}

double von_mises(const Vec6& s)
{
    const double a = s[0] - s[1], b = s[1] - s[2], c = s[2] - s[0];
    return std::sqrt(0.5 * (a * a + b * b + c * c) + 3.0 * (s[3] * s[3] + s[4] * s[4] + s[5] * s[5]));
}

// Geometric multigrid on the voxel node grid, used as the preconditioner of conjugate gradients.
// Coarse levels take every other grid node with trilinear interpolation and Galerkin operators
// (P^T A P), so they need nothing but node coordinates; damped Jacobi smoothing; a direct solve on
// the coarsest level. Cost grows linearly with the model size, unlike a direct factorization, which
// becomes very slow for chunky 3D parts.
class Multigrid
{
public:
    // a_lower: lower triangle of the system matrix. Free dof d is component d % 3 of node d / 3, whose
    // integer grid coordinates are coords[d / 3].
    bool setup(const SpMat& a_lower, std::vector<std::array<int, 3>> coords)
    {
        m_levels.clear();
        SpMat a = a_lower.selfadjointView<Eigen::Lower>();
        for (int depth = 0;; ++depth) {
            Level level;
            level.a        = std::move(a);
            level.inv_diag = level.a.diagonal().cwiseInverse();
            level.omega    = jacobi_weight(level);
            const Eigen::Index n = level.a.rows();
            if (n <= kCoarseDofs || depth >= 12) {
                m_levels.push_back(std::move(level));
                break;
            }
            std::unordered_map<int64_t, int>     coarse_id;
            std::vector<std::array<int, 3>>      coarse;
            std::vector<Eigen::Triplet<double>>  triplets;
            triplets.reserve(size_t(n) * 4);
            for (size_t node = 0; node < coords.size(); ++node) {
                const auto& c = coords[node];
                int         lo[3], cnt[3];
                for (int ax = 0; ax < 3; ++ax) {
                    lo[ax]  = c[ax] / 2;
                    cnt[ax] = (c[ax] % 2 == 0) ? 1 : 2;
                }
                const double w = 1.0 / double(cnt[0] * cnt[1] * cnt[2]);
                for (int dk = 0; dk < cnt[2]; ++dk)
                    for (int dj = 0; dj < cnt[1]; ++dj)
                        for (int di = 0; di < cnt[0]; ++di) {
                            const std::array<int, 3> cc{lo[0] + di, lo[1] + dj, lo[2] + dk};
                            const int64_t key = (int64_t(cc[2]) << 42) | (int64_t(cc[1]) << 21) | int64_t(cc[0]);
                            auto [it, inserted] = coarse_id.emplace(key, int(coarse.size()));
                            if (inserted)
                                coarse.push_back(cc);
                            for (int comp = 0; comp < 3; ++comp)
                                triplets.emplace_back(int(node * 3 + comp), it->second * 3 + comp, w);
                        }
            }
            if (double(coarse.size()) * 3.0 > 0.8 * double(n)) {
                m_levels.push_back(std::move(level));
                break;
            }
            level.p.resize(n, Eigen::Index(coarse.size()) * 3);
            level.p.setFromTriplets(triplets.begin(), triplets.end());
            const SpMat ap = level.a * level.p;
            a              = SpMat(level.p.transpose() * ap);
            m_levels.push_back(std::move(level));
            coords = std::move(coarse);
        }
        // Where the part is ragged, coarse nodes can outnumber the fine ones they interpolate, so P^T A P is
        // only semi-definite. Its null space is that of P and never reaches the fine level; a tiny diagonal
        // shift makes the coarse factorization well defined without changing the correction.
        SpMat coarse = m_levels.back().a;
        for (Eigen::Index i = 0; i < coarse.rows(); ++i)
            coarse.coeffRef(i, i) *= 1.0 + 1e-9;
        m_coarse.compute(coarse);
        return m_coarse.info() == Eigen::Success;
    }

    const SpMat& matrix() const { return m_levels.front().a; }

    // Preconditioned conjugate gradients. x holds the starting guess. Returns the iteration count, or -1.
    int solve(const Eigen::VectorXd& b, Eigen::VectorXd& x, double tolerance, int max_iterations) const
    {
        const SpMat& a      = matrix();
        const double b_norm = b.norm();
        if (x.size() != b.size())
            x = Eigen::VectorXd::Zero(b.size());
        if (b_norm == 0.0) {
            x.setZero();
            return 0;
        }
        Eigen::VectorXd r = b - a * x;
        if (r.norm() <= tolerance * b_norm)
            return 0;
        Eigen::VectorXd z;
        vcycle(0, r, z);
        Eigen::VectorXd p  = z;
        double          rz = r.dot(z);
        for (int it = 1; it <= max_iterations; ++it) {
            const Eigen::VectorXd ap    = a * p;
            const double          alpha = rz / p.dot(ap);
            x += alpha * p;
            r -= alpha * ap;
            if (r.norm() <= tolerance * b_norm)
                return it;
            vcycle(0, r, z);
            const double rz_new = r.dot(z);
            p                   = z + (rz_new / rz) * p;
            rz                  = rz_new;
        }
        return -1;
    }

private:
    static constexpr Eigen::Index kCoarseDofs = 3000;
    static constexpr int          kSweeps     = 2;

    struct Level
    {
        SpMat           a;        // full symmetric matrix
        Eigen::VectorXd inv_diag;
        double          omega = 0.6;
        SpMat           p;        // prolongation from the next coarser level
    };

    static double jacobi_weight(const Level& level)
    {
        // Largest eigenvalue of D^-1 A by power iteration; 4 / (3 lambda) is the classic smoothing weight.
        Eigen::VectorXd x = Eigen::VectorXd::Ones(level.a.rows());
        for (Eigen::Index i = 0; i < x.size(); ++i)
            x[i] += 0.37 * std::sin(double(i));
        double lambda = 1.0;
        for (int it = 0; it < 15; ++it) {
            const Eigen::VectorXd y = level.inv_diag.cwiseProduct(level.a * x);
            lambda                  = y.norm() / std::max(x.norm(), 1e-300);
            x                       = y / std::max(y.norm(), 1e-300);
        }
        return 4.0 / (3.0 * std::max(lambda, 1e-12));
    }

    void vcycle(size_t l, const Eigen::VectorXd& r, Eigen::VectorXd& z) const
    {
        const Level& level = m_levels[l];
        if (l + 1 == m_levels.size()) {
            z = m_coarse.solve(r);
            return;
        }
        z = level.omega * level.inv_diag.cwiseProduct(r);
        for (int s = 1; s < kSweeps; ++s)
            z += level.omega * level.inv_diag.cwiseProduct(r - level.a * z);
        const Eigen::VectorXd rc = level.p.transpose() * (r - level.a * z);
        Eigen::VectorXd       zc;
        vcycle(l + 1, rc, zc);
        z += level.p * zc;
        for (int s = 0; s < kSweeps; ++s)
            z += level.omega * level.inv_diag.cwiseProduct(r - level.a * z);
    }

    std::vector<Level> m_levels;
    Cholesky           m_coarse;
};

enum class BasisKind { Force, Torque, SelfWeight };

constexpr double kTolerance     = 1e-5; // relative residual of each solve
constexpr int    kMaxIterations = 500;

// Identifies a unit load: a unit force or torque on a region, or the part's own weight per unit
// acceleration and unit density.
struct BasisKey
{
    BasisKind kind   = BasisKind::Force;
    Vec3      center = Vec3::Zero();
    double    radius = 0.0;
    Vec3      dir    = Vec3::Zero();
    bool operator==(const BasisKey& o) const
    {
        return kind == o.kind && center == o.center && radius == o.radius && dir == o.dir;
    }
};

} // namespace

struct StructuralModel::Impl
{
    bool   built = false;
    double h     = 1.0;
    Vec3   origin;
    int    nx = 0, ny = 0, nz = 0;
    double volume = 0.0;
    Eigen::AlignedBox3d bbox;

    // Elements (active voxels).
    std::vector<int>                 cell_to_element; // dense, -1 = empty
    std::vector<std::array<int, 3>>  element_ijk;
    std::vector<float>               fill;            // 0..1 share of the voxel inside the part
    std::vector<float>               dist_side;       // mm from voxel centre to the nearest side wall (same layer)
    std::vector<float>               dist_up;         // mm to the nearest top surface above
    std::vector<float>               dist_down;       // mm to the nearest bottom surface below
    std::vector<std::array<int, 8>>  element_nodes;
    std::vector<int64_t>             node_keys;       // sorted grid keys of the nodes
    std::vector<char>                element_kept;    // connected to a support

    Mat24   k_unit;
    Mat6x24 b_unit;
    Mat6    d_unit;

    Material          material;
    StructureSettings structure;
    std::vector<double> rel_stiffness;  // element stiffness / solid stiffness, includes fill
    std::vector<double> solid_share;    // share of the voxel that is walls or shells

    // Boundary conditions and system.
    std::vector<char>   node_fixed;
    std::vector<char>   node_active;
    std::vector<int>    dof_to_free;
    int                 num_free     = 0;
    int                 num_ignored  = 0;
    bool                pattern_ok   = false;
    std::vector<int>    col_ptr;
    std::vector<int>    row_idx;
    std::vector<double> node_attached_kg;
    std::vector<RegionMass> attached;

    SpMat     k_current; // lower triangle, unit modulus
    bool      k_current_ok = false;
    Multigrid solver;
    bool      solver_ok = false;

    // Responses to unit loads for the current structure (unit modulus). Every load case is a weighted
    // sum of them, so changing a motor, a propeller or a force magnitude never needs a new solve.
    struct Basis
    {
        BasisKey        key;
        Eigen::VectorXd u;
    };
    std::vector<Basis> basis;
    std::vector<Basis> stale_basis; // responses for the previous walls/infill: starting guesses

    int64_t key(int i, int j, int k) const { return (int64_t(k) * (ny + 1) + j) * (nx + 1) + i; }
    int     node_of_key(int64_t k) const
    {
        auto it = std::lower_bound(node_keys.begin(), node_keys.end(), k);
        return (it != node_keys.end() && *it == k) ? int(it - node_keys.begin()) : -1;
    }
    Vec3 node_pos(int n) const
    {
        const int64_t k  = node_keys[n];
        const int     i  = int(k % (nx + 1));
        const int     j  = int((k / (nx + 1)) % (ny + 1));
        const int     kk = int(k / (int64_t(nx + 1) * (ny + 1)));
        return origin + h * Vec3(i, j, kk);
    }
    Vec3 element_pos(int e) const
    {
        const auto& c = element_ijk[e];
        return origin + h * Vec3(c[0] + 0.5, c[1] + 0.5, c[2] + 0.5);
    }

    double solid_share_for(int e, const StructureSettings& s) const
    {
        const double side = band_fraction(dist_side[e], s.wall_thickness(), h);
        const double up   = band_fraction(dist_up[e], s.top_thickness(), h);
        const double down = band_fraction(dist_down[e], s.bottom_thickness(), h);
        return std::max(side, std::max(up, down));
    }

    void update_element_properties()
    {
        const double e_inf = infill_relative_stiffness(structure);
        rel_stiffness.resize(fill.size());
        solid_share.resize(fill.size());
        for (size_t e = 0; e < fill.size(); ++e) {
            const double phi = solid_share_for(int(e), structure);
            solid_share[e]   = phi;
            // Floor keeps hollow (0% infill) interiors from making the system singular.
            rel_stiffness[e] = fill[e] * std::max(phi + (1.0 - phi) * e_inf, 1e-4);
        }
        k_current_ok = false;
        solver_ok    = false;
        if (!basis.empty())
            stale_basis = std::move(basis);
        basis.clear();
    }

    std::vector<int> nodes_in(const Region& r) const
    {
        std::vector<int> out;
        const double     r2 = r.radius * r.radius;
        for (int n = 0; n < int(node_keys.size()); ++n)
            if (node_active.empty() || node_active[n])
                if ((node_pos(n) - r.center).squaredNorm() <= r2)
                    out.push_back(n);
        if (out.empty()) {
            // The point is off the part: take the closest nodes so a slightly misplaced load still lands.
            std::vector<std::pair<double, int>> d;
            for (int n = 0; n < int(node_keys.size()); ++n)
                if (node_active.empty() || node_active[n])
                    d.emplace_back((node_pos(n) - r.center).squaredNorm(), n);
            const size_t take = std::min<size_t>(8, d.size());
            std::partial_sort(d.begin(), d.begin() + take, d.end());
            for (size_t i = 0; i < take; ++i)
                out.push_back(d[i].second);
        }
        return out;
    }

    std::array<int, 3> node_ijk(int n) const
    {
        const int64_t k = node_keys[n];
        return {int(k % (nx + 1)), int((k / (nx + 1)) % (ny + 1)), int(k / (int64_t(nx + 1) * (ny + 1)))};
    }

    void build_pattern()
    {
        // Pieces of the part not face-connected to a support would float; drop them.
        const int ne = int(element_ijk.size());
        std::vector<int> parent(ne);
        std::iota(parent.begin(), parent.end(), 0);
        auto find = [&](int x) {
            while (parent[x] != x)
                x = parent[x] = parent[parent[x]];
            return x;
        };
        for (int e = 0; e < ne; ++e) {
            const auto& c = element_ijk[e];
            const int   nb[3][3] = {{c[0] + 1, c[1], c[2]}, {c[0], c[1] + 1, c[2]}, {c[0], c[1], c[2] + 1}};
            for (const auto& q : nb) {
                if (q[0] >= nx || q[1] >= ny || q[2] >= nz)
                    continue;
                const int other = cell_to_element[(size_t(q[2]) * ny + q[1]) * nx + q[0]];
                if (other >= 0)
                    parent[find(e)] = find(other);
            }
        }
        std::vector<char> root_supported(ne, 0);
        for (int e = 0; e < ne; ++e)
            for (int n : element_nodes[e])
                if (node_fixed[n])
                    root_supported[find(e)] = 1;
        element_kept.assign(ne, 0);
        num_ignored = 0;
        node_active.assign(node_keys.size(), 0);
        for (int e = 0; e < ne; ++e) {
            element_kept[e] = root_supported[find(e)];
            if (element_kept[e])
                for (int n : element_nodes[e])
                    node_active[n] = 1;
            else
                ++num_ignored;
        }

        const int nn = int(node_keys.size());
        // Free dofs grouped per node (x, y, z), as the multigrid expects.
        dof_to_free.assign(size_t(nn) * 3, -1);
        num_free = 0;
        for (int n = 0; n < nn; ++n)
            if (node_active[n] && !node_fixed[n])
                for (int c = 0; c < 3; ++c)
                    dof_to_free[size_t(n) * 3 + c] = num_free++;

        std::vector<std::vector<int>> adj(nn);
        for (int e = 0; e < ne; ++e) {
            if (!element_kept[e])
                continue;
            for (int a : element_nodes[e])
                for (int b : element_nodes[e])
                    adj[a].push_back(b);
        }
        for (auto& list : adj) {
            std::sort(list.begin(), list.end());
            list.erase(std::unique(list.begin(), list.end()), list.end());
        }
        // Lower triangle in compressed columns: rows >= column, sorted, diagonal first.
        col_ptr.assign(size_t(num_free) + 1, 0);
        for (int n = 0; n < nn; ++n)
            for (int c = 0; c < 3; ++c) {
                const int col = dof_to_free[size_t(n) * 3 + c];
                if (col < 0)
                    continue;
                int count = 0;
                for (int m2 : adj[n])
                    for (int c2 = 0; c2 < 3; ++c2)
                        if (dof_to_free[size_t(m2) * 3 + c2] >= col)
                            ++count;
                col_ptr[col + 1] = count;
            }
        for (int col = 0; col < num_free; ++col)
            col_ptr[col + 1] += col_ptr[col];
        row_idx.assign(size_t(col_ptr[num_free]), 0);
        for (int n = 0; n < nn; ++n)
            for (int c = 0; c < 3; ++c) {
                const int col = dof_to_free[size_t(n) * 3 + c];
                if (col < 0)
                    continue;
                int* out = row_idx.data() + col_ptr[col];
                for (int m2 : adj[n])
                    for (int c2 = 0; c2 < 3; ++c2) {
                        const int row = dof_to_free[size_t(m2) * 3 + c2];
                        if (row >= col)
                            *out++ = row;
                    }
                std::sort(row_idx.data() + col_ptr[col], out);
            }
        pattern_ok   = true;
        k_current_ok = false;
        solver_ok    = false;
        basis.clear();
        stale_basis.clear();
        rebuild_attached_masses();
    }

    void assemble()
    {
        if (k_current_ok)
            return;
        std::vector<double> values(row_idx.size(), 0.0);
        double              diag_sum = 0.0;
        for (size_t e = 0; e < element_ijk.size(); ++e) {
            if (!element_kept[e])
                continue;
            const double scale = rel_stiffness[e] * h;
            const auto&  nodes = element_nodes[e];
            for (int b = 0; b < 8; ++b)
                for (int cb = 0; cb < 3; ++cb) {
                    const int col = dof_to_free[size_t(nodes[b]) * 3 + cb];
                    if (col < 0)
                        continue;
                    const int* begin = row_idx.data() + col_ptr[col];
                    const int* end   = row_idx.data() + col_ptr[col + 1];
                    for (int a = 0; a < 8; ++a)
                        for (int ca = 0; ca < 3; ++ca) {
                            const int row = dof_to_free[size_t(nodes[a]) * 3 + ca];
                            if (row < col)
                                continue;
                            const int* it = std::lower_bound(begin, end, row);
                            values[size_t(it - row_idx.data())] += scale * k_unit(3 * a + ca, 3 * b + cb);
                        }
                }
        }
        for (int col = 0; col < num_free; ++col)
            diag_sum += values[col_ptr[col]];
        // A whisper of stiffness on the diagonal guards against hinge-like voxel contacts.
        const double eps = 1e-9 * (num_free > 0 ? diag_sum / num_free : 1.0);
        for (int col = 0; col < num_free; ++col)
            values[col_ptr[col]] += eps;
        Eigen::Map<const SpMat> map(num_free, num_free, int(values.size()), col_ptr.data(), row_idx.data(), values.data());
        k_current    = map;
        k_current_ok = true;
    }

    bool prepare(std::string* error)
    {
        if (solver_ok)
            return true;
        if (!pattern_ok || num_free == 0) {
            if (error)
                *error = "Add at least one support (a place where the part is held) before analysing.";
            return false;
        }
        assemble();
        std::vector<std::array<int, 3>> coords(size_t(num_free) / 3);
        for (int n = 0; n < int(node_keys.size()); ++n) {
            const int dof = dof_to_free[size_t(n) * 3];
            if (dof >= 0)
                coords[size_t(dof) / 3] = node_ijk(n);
        }
        solver_ok = solver.setup(k_current, std::move(coords));
        if (!solver_ok && error)
            *error = "The part could not be analysed; check that the supports hold it.";
        return solver_ok;
    }

    void rebuild_attached_masses()
    {
        node_attached_kg.assign(node_keys.size(), 0.0);
        for (const RegionMass& rm : attached) {
            std::vector<int> nodes = nodes_in(rm.region);
            if (nodes.empty())
                continue;
            for (int n : nodes)
                node_attached_kg[n] += rm.mass_g * 1e-3 / double(nodes.size());
        }
    }

    double element_mass_kg(size_t e) const
    {
        return element_volume_fraction(e) * h * h * h * material.density * 1e-6; // g/cm3 = 1e-6 kg/mm3
    }

    Eigen::VectorXd lumped_mass() const
    {
        Eigen::VectorXd mass = Eigen::VectorXd::Zero(num_free);
        auto add = [&](int node, double kg) {
            for (int c = 0; c < 3; ++c) {
                const int dof = dof_to_free[size_t(node) * 3 + c];
                if (dof >= 0)
                    mass[dof] += kg;
            }
        };
        for (size_t e = 0; e < element_ijk.size(); ++e)
            if (element_kept[e]) {
                const double m8 = element_mass_kg(e) / 8.0;
                for (int n : element_nodes[e])
                    add(n, m8);
            }
        for (size_t n = 0; n < node_attached_kg.size(); ++n)
            if (node_attached_kg[n] > 0)
                add(int(n), node_attached_kg[n]);
        return mass;
    }

    Eigen::VectorXd basis_load(const BasisKey& key) const
    {
        Eigen::VectorXd f = Eigen::VectorXd::Zero(num_free);
        auto add = [&](int node, const Vec3& force) {
            for (int c = 0; c < 3; ++c) {
                const int dof = dof_to_free[size_t(node) * 3 + c];
                if (dof >= 0)
                    f[dof] += force[c];
            }
        };
        switch (key.kind) {
        case BasisKind::Force: {
            const std::vector<int> nodes = nodes_in(Region{key.center, key.radius});
            for (int n : nodes)
                add(n, key.dir / double(nodes.size()));
            break;
        }
        case BasisKind::Torque: {
            const std::vector<int> nodes = nodes_in(Region{key.center, key.radius});
            std::vector<Vec3>      arms;
            double                 sum = 0.0;
            for (int n : nodes) {
                Vec3 r = node_pos(n) - key.center;
                r -= r.dot(key.dir) * key.dir;
                arms.push_back(r);
                sum += r.squaredNorm();
            }
            if (sum > 1e-12)
                for (size_t i = 0; i < nodes.size(); ++i)
                    add(nodes[i], key.dir.cross(arms[i]) / sum);
            break;
        }
        case BasisKind::SelfWeight:
            // Weight of the part at 1 m/s2 and 1 g/cm3, in N.
            for (size_t e = 0; e < element_ijk.size(); ++e)
                if (element_kept[e]) {
                    const Vec3 fe = key.dir * (element_volume_fraction(e) * h * h * h * 1e-6 / 8.0);
                    for (int n : element_nodes[e])
                        add(n, fe);
                }
            break;
        }
        return f;
    }

    int find_response(const BasisKey& key) const
    {
        for (size_t i = 0; i < basis.size(); ++i)
            if (basis[i].key == key)
                return int(i);
        return -1;
    }

    // Solves for every response in `keys` not cached yet, in parallel (the solves are independent and
    // the multigrid is read-only while solving).
    bool ensure_responses(const std::vector<BasisKey>& keys, int& iterations, std::string* error)
    {
        std::vector<BasisKey> missing;
        for (const BasisKey& key : keys)
            if (find_response(key) < 0 && std::find(missing.begin(), missing.end(), key) == missing.end())
                missing.push_back(key);
        if (missing.empty())
            return true;
        if (!prepare(error))
            return false;
        std::vector<Eigen::VectorXd> solutions(missing.size());
        std::vector<int>             counts(missing.size(), 0);
        for (size_t i = 0; i < missing.size(); ++i)
            for (const Basis& old : stale_basis)
                if (old.key == missing[i] && old.u.size() == num_free)
                    solutions[i] = old.u;
        std::atomic<size_t> next{0};
        auto worker = [&]() {
            for (size_t i = next++; i < missing.size(); i = next++)
                counts[i] = solver.solve(basis_load(missing[i]), solutions[i], kTolerance, kMaxIterations);
        };
        const size_t threads = std::min<size_t>(missing.size(), std::max(1u, std::thread::hardware_concurrency()));
        std::vector<std::thread> pool;
        for (size_t t = 1; t < threads; ++t)
            pool.emplace_back(worker);
        worker();
        for (std::thread& t : pool)
            t.join();
        for (size_t i = 0; i < missing.size(); ++i) {
            if (counts[i] < 0) {
                if (error)
                    *error = "The analysis did not converge; the part may be barely connected to its supports.";
                return false;
            }
            iterations += counts[i];
            basis.push_back({missing[i], std::move(solutions[i])});
        }
        return true;
    }

    // Share of a voxel occupied by deposited plastic for the current structure.
    double element_volume_fraction(size_t e) const
    {
        const double phi = solid_share[e];
        return fill[e] * (phi + (1.0 - phi) * std::clamp(structure.infill_density, 0.0, 1.0));
    }
};

StructuralModel::StructuralModel() : m(std::make_unique<Impl>()) {}
StructuralModel::~StructuralModel() = default;

bool StructuralModel::is_built() const { return m->built; }
double StructuralModel::element_size() const { return m->h; }
size_t StructuralModel::num_elements() const { return m->element_ijk.size(); }
size_t StructuralModel::num_nodes() const { return m->node_keys.size(); }
Vec3 StructuralModel::element_center(size_t e) const { return m->element_pos(int(e)); }
Vec3 StructuralModel::node_position(size_t n) const { return m->node_pos(int(n)); }
double StructuralModel::part_volume() const { return m->volume; }
Eigen::AlignedBox3d StructuralModel::bounding_box() const { return m->bbox; }
int StructuralModel::num_ignored_elements() const { return m->num_ignored; }
const Material& StructuralModel::material() const { return m->material; }
const StructureSettings& StructuralModel::structure() const { return m->structure; }
bool StructuralModel::solver_ready() const { return m->solver_ok; }

bool StructuralModel::build(const MeshView& mesh, const MeshOptions& options, std::string* error)
{
    m = std::make_unique<Impl>();
    if (mesh.num_faces == 0 || mesh.num_vertices == 0) {
        if (error)
            *error = "The object has no triangles.";
        return false;
    }
    auto vtx = [&](int i) {
        return Vec3(mesh.vertices[3 * size_t(i)], mesh.vertices[3 * size_t(i) + 1], mesh.vertices[3 * size_t(i) + 2]);
    };
    m->bbox.setEmpty();
    for (size_t i = 0; i < mesh.num_vertices; ++i)
        m->bbox.extend(vtx(int(i)));
    double signed_volume = 0.0;
    for (size_t f = 0; f < mesh.num_faces; ++f) {
        const Vec3 a = vtx(mesh.indices[3 * f]), b = vtx(mesh.indices[3 * f + 1]), c = vtx(mesh.indices[3 * f + 2]);
        signed_volume += a.dot(b.cross(c)) / 6.0;
    }
    const double volume = std::abs(signed_volume);
    if (volume <= 1e-6) {
        if (error)
            *error = "The object is not a closed solid.";
        return false;
    }
    // Outward normals give a positive signed volume; an inside-out mesh flips the crossing direction.
    const double orientation = signed_volume >= 0 ? 1.0 : -1.0;

    const Vec3 extent = m->bbox.sizes();
    double     h      = options.element_size > 0 ? options.element_size
                                                 : std::cbrt(volume / std::max(1000, options.target_elements));
    h = std::max({h, extent.maxCoeff() / std::max(8, options.max_cells_per_axis), 0.2});
    // Keep the dense voxel grid bounded in memory.
    while ((extent.x() / h + 3) * (extent.y() / h + 3) * (extent.z() / h + 3) > 3.0e7)
        h *= 1.15;
    m->h      = h;
    m->origin = m->bbox.min() - Vec3::Constant(h);
    m->nx     = int(std::ceil(extent.x() / h)) + 2;
    m->ny     = int(std::ceil(extent.y() / h)) + 2;
    m->nz     = int(std::ceil(extent.z() / h)) + 2;
    const int nx = m->nx, ny = m->ny, nz = m->nz;

    // --- Fill fraction by casting kSubRays^2 vertical rays per column. ---
    const int sx = nx * kSubRays, sy = ny * kSubRays;
    std::vector<std::vector<std::pair<double, double>>> hits(size_t(sx) * sy); // (z, +1 entering / -1 leaving)
    const double jitter_x = 1.37e-4 * h, jitter_y = 2.71e-4 * h;
    auto ray_x = [&](int s) { return m->origin.x() + (s + 0.5) * h / kSubRays + jitter_x; };
    auto ray_y = [&](int s) { return m->origin.y() + (s + 0.5) * h / kSubRays + jitter_y; };
    for (size_t f = 0; f < mesh.num_faces; ++f) {
        const Vec3   a = vtx(mesh.indices[3 * f]), b = vtx(mesh.indices[3 * f + 1]), c = vtx(mesh.indices[3 * f + 2]);
        const double area2 = (b.x() - a.x()) * (c.y() - a.y()) - (c.x() - a.x()) * (b.y() - a.y());
        if (std::abs(area2) < 1e-14)
            continue; // vertical triangle, no vertical ray crosses it
        // Normal pointing down (area2 < 0 for outward orientation) means the ray enters the solid.
        const double sign = (area2 < 0 ? 1.0 : -1.0) * orientation;
        const double minx = std::min({a.x(), b.x(), c.x()}), maxx = std::max({a.x(), b.x(), c.x()});
        const double miny = std::min({a.y(), b.y(), c.y()}), maxy = std::max({a.y(), b.y(), c.y()});
        const int    s0 = std::max(0, int(std::floor((minx - m->origin.x()) / h * kSubRays - 0.5)));
        const int    s1 = std::min(sx - 1, int(std::ceil((maxx - m->origin.x()) / h * kSubRays)));
        const int    t0 = std::max(0, int(std::floor((miny - m->origin.y()) / h * kSubRays - 0.5)));
        const int    t1 = std::min(sy - 1, int(std::ceil((maxy - m->origin.y()) / h * kSubRays)));
        for (int t = t0; t <= t1; ++t) {
            const double y = ray_y(t);
            for (int s = s0; s <= s1; ++s) {
                const double x  = ray_x(s);
                const double w0 = ((b.x() - x) * (c.y() - y) - (c.x() - x) * (b.y() - y)) / area2;
                const double w1 = ((c.x() - x) * (a.y() - y) - (a.x() - x) * (c.y() - y)) / area2;
                const double w2 = 1.0 - w0 - w1;
                if (w0 < 0 || w1 < 0 || w2 < 0)
                    continue;
                hits[size_t(t) * sx + s].emplace_back(w0 * a.z() + w1 * b.z() + w2 * c.z(), sign);
            }
        }
    }
    std::vector<float> cell_fill(size_t(nx) * ny * nz, 0.f);
    const double       per_ray = 1.0 / (kSubRays * kSubRays * h);
    for (int t = 0; t < sy; ++t)
        for (int s = 0; s < sx; ++s) {
            auto& list = hits[size_t(t) * sx + s];
            if (list.empty())
                continue;
            std::sort(list.begin(), list.end());
            const int i = s / kSubRays, j = t / kSubRays;
            double    winding = 0.0, enter_z = 0.0;
            for (const auto& [z, w] : list) {
                const bool was_inside = winding > 0.5;
                winding += w;
                const bool inside = winding > 0.5;
                if (!was_inside && inside)
                    enter_z = z;
                else if (was_inside && !inside) {
                    // Spread the interval [enter_z, z] over the voxels of this column.
                    const double z0 = (enter_z - m->origin.z()) / h, z1 = (z - m->origin.z()) / h;
                    for (int k = std::max(0, int(std::floor(z0))); k <= std::min(nz - 1, int(std::floor(z1))); ++k) {
                        const double overlap = std::min(z1, k + 1.0) - std::max(z0, double(k));
                        if (overlap > 0)
                            cell_fill[(size_t(k) * ny + j) * nx + i] += float(overlap * h * per_ray);
                    }
                }
            }
        }
    hits.clear();
    hits.shrink_to_fit();

    // --- Elements. ---
    m->cell_to_element.assign(cell_fill.size(), -1);
    for (int k = 0; k < nz; ++k)
        for (int j = 0; j < ny; ++j)
            for (int i = 0; i < nx; ++i) {
                const size_t c = (size_t(k) * ny + j) * nx + i;
                const float  f = std::min(cell_fill[c], 1.f);
                if (f >= options.min_fill) {
                    m->cell_to_element[c] = int(m->element_ijk.size());
                    m->element_ijk.push_back({i, j, k});
                    m->fill.push_back(f);
                }
            }
    if (m->element_ijk.size() < 8) {
        if (error)
            *error = "The object is too small or too thin to analyse at this resolution.";
        return false;
    }
    m->volume = 0.0;
    for (float f : m->fill)
        m->volume += f * h * h * h;

    // --- Distances from each voxel to the part surface, for walls and top/bottom shells. ---
    std::vector<double> sq(cell_fill.size());
    for (size_t c = 0; c < cell_fill.size(); ++c)
        sq[c] = cell_fill[c] >= 0.5f ? kInf : 0.0;
    {
        std::vector<int>    v;
        std::vector<double> z, in, out;
        for (int k = 0; k < nz; ++k) {
            in.resize(nx);
            out.resize(nx);
            for (int j = 0; j < ny; ++j) {
                double* row = &sq[(size_t(k) * ny + j) * nx];
                std::copy(row, row + nx, in.begin());
                distance_transform_1d(in.data(), nx, out.data(), v, z);
                std::copy(out.begin(), out.end(), row);
            }
            in.resize(ny);
            out.resize(ny);
            for (int i = 0; i < nx; ++i) {
                for (int j = 0; j < ny; ++j)
                    in[j] = sq[(size_t(k) * ny + j) * nx + i];
                distance_transform_1d(in.data(), ny, out.data(), v, z);
                for (int j = 0; j < ny; ++j)
                    sq[(size_t(k) * ny + j) * nx + i] = out[j];
            }
        }
    }
    const size_t ne = m->element_ijk.size();
    m->dist_side.resize(ne);
    m->dist_up.resize(ne);
    m->dist_down.resize(ne);
    auto inside_cell = [&](int i, int j, int k) {
        return k >= 0 && k < nz && cell_fill[(size_t(k) * ny + j) * nx + i] >= 0.5f;
    };
    for (size_t e = 0; e < ne; ++e) {
        const auto [i, j, k] = m->element_ijk[e];
        const size_t c       = (size_t(k) * ny + j) * nx + i;
        if (cell_fill[c] < 0.5f) {
            m->dist_side[e] = m->dist_up[e] = m->dist_down[e] = float(-0.5 * h);
            continue;
        }
        m->dist_side[e] = float((std::sqrt(sq[c]) - 0.5) * h);
        int up = 1;
        while (inside_cell(i, j, k + up))
            ++up;
        int down = 1;
        while (inside_cell(i, j, k - down))
            ++down;
        m->dist_up[e]   = float((up - 0.5) * h);
        m->dist_down[e] = float((down - 0.5) * h);
    }

    // --- Nodes. ---
    m->node_keys.reserve(ne * 2);
    for (const auto& c : m->element_ijk)
        for (const auto& o : kCorner)
            m->node_keys.push_back(m->key(c[0] + o[0], c[1] + o[1], c[2] + o[2]));
    std::sort(m->node_keys.begin(), m->node_keys.end());
    m->node_keys.erase(std::unique(m->node_keys.begin(), m->node_keys.end()), m->node_keys.end());
    m->element_nodes.resize(ne);
    for (size_t e = 0; e < ne; ++e) {
        const auto& c = m->element_ijk[e];
        for (int a = 0; a < 8; ++a)
            m->element_nodes[e][a] = m->node_of_key(m->key(c[0] + kCorner[a][0], c[1] + kCorner[a][1], c[2] + kCorner[a][2]));
    }
    m->node_fixed.assign(m->node_keys.size(), 0);

    unit_brick(options.poisson, m->k_unit, m->b_unit, m->d_unit);
    m->update_element_properties();
    m->built = true;
    return true;
}

int StructuralModel::set_supports(const std::vector<Region>& fixed)
{
    m->node_fixed.assign(m->node_keys.size(), 0);
    m->node_active.clear();
    int count = 0;
    for (const Region& r : fixed)
        for (int n : m->nodes_in(r))
            if (!m->node_fixed[n]) {
                m->node_fixed[n] = 1;
                ++count;
            }
    m->build_pattern();
    return count;
}

void StructuralModel::set_attached_masses(const std::vector<RegionMass>& masses)
{
    m->attached = masses;
    m->rebuild_attached_masses();
}

void StructuralModel::set_material(const Material& material) { m->material = material; }

void StructuralModel::set_structure(const StructureSettings& structure)
{
    if (structure == m->structure && !m->rel_stiffness.empty())
        return;
    m->structure = structure;
    m->update_element_properties();
}

bool StructuralModel::prepare_solver(std::string* error) { return m->prepare(error); }

double StructuralModel::estimated_mass_g() const { return estimated_mass_g(m->structure, m->material); }

double StructuralModel::extruded_volume(const StructureSettings& s) const
{
    const double rho    = std::clamp(s.infill_density, 0.0, 1.0);
    double       volume = 0.0;
    for (size_t e = 0; e < m->fill.size(); ++e) {
        const double phi = m->solid_share_for(int(e), s);
        volume += m->fill[e] * (phi + (1.0 - phi) * rho);
    }
    return volume * m->h * m->h * m->h;
}

double StructuralModel::estimated_mass_g(const StructureSettings& s, const Material& material) const
{
    return extruded_volume(s) * material.density * 1e-3; // g/cm3 = 1e-3 g/mm3
}

CaseResult StructuralModel::solve(const LoadCase& lc)
{
    CaseResult res;
    res.id = lc.id;
    if (!m->built || !m->pattern_ok || m->num_free == 0) {
        res.error = "Add at least one support (a place where the part is held) before analysing.";
        return res;
    }

    // --- Displacement for unit modulus as a weighted sum of cached unit-load responses. ---
    std::vector<std::pair<BasisKey, double>> terms;
    const Vec3 axes[3] = {Vec3::UnitX(), Vec3::UnitY(), Vec3::UnitZ()};
    for (const RegionForce& rf : lc.forces)
        for (int a = 0; a < 3; ++a)
            if (rf.force[a] != 0.0)
                terms.push_back({BasisKey{BasisKind::Force, rf.region.center, rf.region.radius, axes[a]}, rf.force[a]});
    for (const RegionTorque& rt : lc.torques)
        if (rt.torque != 0.0 && rt.axis.norm() > 0)
            terms.push_back({BasisKey{BasisKind::Torque, rt.region.center, rt.region.radius, rt.axis.normalized()}, rt.torque});
    for (int a = 0; a < 3; ++a) {
        const double accel = lc.acceleration_g[a] * kStandardG; // m/s2
        if (accel == 0.0)
            continue;
        terms.push_back({BasisKey{BasisKind::SelfWeight, Vec3::Zero(), 0.0, axes[a]}, accel * m->material.density});
        for (const RegionMass& rm : m->attached)
            if (rm.mass_g > 0)
                terms.push_back({BasisKey{BasisKind::Force, rm.region.center, rm.region.radius, axes[a]}, accel * rm.mass_g * 1e-3});
    }
    std::vector<BasisKey> keys;
    for (const auto& term : terms)
        keys.push_back(term.first);
    if (!m->ensure_responses(keys, res.iterations, &res.error))
        return res;
    Eigen::VectorXd u_unit = Eigen::VectorXd::Zero(m->num_free);
    for (const auto& [key, coef] : terms)
        u_unit += coef * m->basis[size_t(m->find_response(key))].u;
    const double modulus = std::max(m->material.modulus_xy, 1e-6);

    // --- Displacements. ---
    const size_t nn = m->node_keys.size();
    res.node_displacement.assign(nn, 0.f);
    for (size_t n = 0; n < nn; ++n) {
        Vec3 d = Vec3::Zero();
        for (int c = 0; c < 3; ++c) {
            const int dof = m->dof_to_free[n * 3 + c];
            if (dof >= 0)
                d[c] = u_unit[dof] / modulus;
        }
        res.node_displacement[n] = float(d.norm());
        if (d.norm() > res.max_displacement) {
            res.max_displacement       = d.norm();
            res.max_displacement_point = m->node_pos(int(n));
        }
    }

    // --- Stresses and safety factors. ---
    const double e_inf   = infill_relative_stiffness(m->structure);
    const double s_inf   = std::max(infill_relative_strength(m->structure), 1e-6);
    const double sxy     = std::max(m->material.tensile_xy, 1e-6);
    const double sz      = std::max(m->material.tensile_z, 1e-6);
    const size_t ne      = m->element_ijk.size();
    res.element_safety.assign(ne, float(kMaxSafety));
    res.element_stress.assign(ne, 0.f);
    res.min_safety_factor = kMaxSafety;
    for (size_t e = 0; e < ne; ++e) {
        if (!m->element_kept[e])
            continue;
        Vec24 ue;
        bool  touches_support = false;
        for (int a = 0; a < 8; ++a) {
            const int n = m->element_nodes[e][a];
            touches_support |= m->node_fixed[n] != 0;
            for (int c = 0; c < 3; ++c) {
                const int dof = m->dof_to_free[size_t(n) * 3 + c];
                ue[3 * a + c] = dof >= 0 ? u_unit[dof] : 0.0;
            }
        }
        // Homogenized stress of the voxel's printed material (walls + infill), independent of modulus.
        const double phi    = m->solid_share[e];
        const double e_hom  = std::max(phi + (1.0 - phi) * e_inf, 1e-4);
        const Vec6   stress = e_hom * (m->d_unit * (m->b_unit * ue)) / m->h;
        // Strain-based check of each constituent: walls (solid plastic) and infill strands.
        double weakness = phi > 1e-3 ? 1.0 : 0.0;
        if (phi < 0.999 && e_inf > 0)
            weakness = std::max(weakness, e_inf / s_inf);
        const double scale     = weakness / e_hom;
        const double vm        = von_mises(stress);
        const double inplane   = vm * scale / sxy;
        const double tension_z = std::max(stress[2], 0.0);
        const double shear_z   = std::sqrt(stress[4] * stress[4] + stress[5] * stress[5]);
        const double layers    = std::hypot(tension_z, shear_z / kShearFactor) * scale / sz;
        const double index     = std::max(inplane, layers);
        const double safety    = index > 1.0 / kMaxSafety ? 1.0 / index : kMaxSafety;
        res.element_safety[e]  = float(safety);
        res.element_stress[e]  = float(vm / e_hom);
        if (touches_support)
            continue; // clamped edges carry artificial stress peaks; report the part itself
        res.max_wall_stress = std::max(res.max_wall_stress, vm / e_hom);
        if (safety < res.min_safety_factor) {
            res.min_safety_factor  = safety;
            res.weakest_point      = m->element_pos(int(e));
            res.interlayer_governs = layers > inplane;
        }
    }
    res.ok = true;
    return res;
}

Vec3 StructuralModel::natural_frequencies()
{
    Vec3 out = Vec3::Zero();
    if (!m->built || !m->pattern_ok || m->num_free < 6)
        return out;
    const Vec3 axes[3] = {Vec3::UnitX(), Vec3::UnitY(), Vec3::UnitZ()};
    std::vector<BasisKey> keys;
    for (const Vec3& axis : axes) {
        keys.push_back({BasisKind::SelfWeight, Vec3::Zero(), 0.0, axis});
        for (const RegionMass& rm : m->attached)
            if (rm.mass_g > 0)
                keys.push_back({BasisKind::Force, rm.region.center, rm.region.radius, axis});
    }
    int iterations = 0;
    if (!m->ensure_responses(keys, iterations, nullptr))
        return out;
    const Eigen::VectorXd mass    = m->lumped_mass(); // kg per dof
    const double          modulus = std::max(m->material.modulus_xy, 1e-6);
    for (int a = 0; a < 3; ++a) {
        // Static deflection under 1 m/s2 along the axis (forces in N), in metres.
        Eigen::VectorXd u = m->material.density * m->basis[size_t(m->find_response({BasisKind::SelfWeight, Vec3::Zero(), 0.0, axes[a]}))].u;
        for (const RegionMass& rm : m->attached)
            if (rm.mass_g > 0)
                u += rm.mass_g * 1e-3 * m->basis[size_t(m->find_response({BasisKind::Force, rm.region.center, rm.region.radius, axes[a]}))].u;
        u *= 1e-3 / modulus;
        // Rayleigh quotient with the deflected shape: omega^2 = (F . u) / (u^T M u), where F = M * 1 m/s2.
        Eigen::VectorXd force = Eigen::VectorXd::Zero(m->num_free);
        for (int dof = a; dof < m->num_free; dof += 3)
            force[dof] = mass[dof];
        const double work = force.dot(u);
        const double kin  = u.cwiseProduct(u).dot(mass);
        if (work > 0 && kin > 0)
            out[a] = std::sqrt(work / kin) / (2.0 * kPi);
    }
    return out;
}

std::vector<float> StructuralModel::sample_elements(const std::vector<float>& values,
                                                    const float*              points,
                                                    size_t                    num_points,
                                                    bool                      take_min) const
{
    std::vector<float> out(num_points, 0.f);
    if (!m->built || values.size() != m->element_ijk.size())
        return out;
    const double h = m->h;
    for (size_t p = 0; p < num_points; ++p) {
        const Vec3 pt(points[3 * p], points[3 * p + 1], points[3 * p + 2]);
        const Vec3 g  = (pt - m->origin) / h;
        const int  ci = int(std::floor(g.x())), cj = int(std::floor(g.y())), ck = int(std::floor(g.z()));
        double     acc = 0.0, wsum = 0.0, best = std::numeric_limits<double>::max();
        for (int radius = 1; radius <= 2 && wsum == 0.0; ++radius)
            for (int dk = -radius; dk <= radius; ++dk)
                for (int dj = -radius; dj <= radius; ++dj)
                    for (int di = -radius; di <= radius; ++di) {
                        const int i = ci + di, j = cj + dj, k = ck + dk;
                        if (i < 0 || j < 0 || k < 0 || i >= m->nx || j >= m->ny || k >= m->nz)
                            continue;
                        const int e = m->cell_to_element[(size_t(k) * m->ny + j) * m->nx + i];
                        if (e < 0 || (!m->element_kept.empty() && !m->element_kept[e]))
                            continue;
                        const double d2 = (m->element_pos(e) - pt).squaredNorm() / (h * h);
                        if (d2 > double(radius) * radius * 1.5)
                            continue;
                        const double w = 1.0 / (0.25 + d2);
                        acc += w * values[e];
                        wsum += w;
                        best = std::min(best, double(values[e]));
                    }
        out[p] = wsum > 0 ? float(take_min ? best : acc / wsum) : (take_min ? float(kMaxSafety) : 0.f);
    }
    return out;
}

}} // namespace Slic3r::SmartPrint
