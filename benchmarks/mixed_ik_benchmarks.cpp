#include <benchmark/benchmark.h>

#include "urdfx/inverse_kinematics.h"
#include "urdfx/kinematics.h"
#include "urdfx/logging.h"
#include "urdfx/urdf_parser.h"

#include <Eigen/Geometry>
#include <algorithm>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <numbers>
#include <stdexcept>
#include <vector>
#include <string>

using namespace urdfx;

namespace
{

    struct BenchmarkCase
    {
        Transform target_pose;
        Eigen::VectorXd initial_guess;
        Eigen::VectorXd ground_truth;
    };

    struct MixedDataset
    {
        int dof;
        std::vector<BenchmarkCase> cases;
    };

    MixedDataset loadDataset(const std::filesystem::path &path)
    {
        std::ifstream file(path, std::ios::binary);
        if (!file)
        {
            throw std::runtime_error("Failed to open dataset: " + path.string());
        }

        char magic[4];
        file.read(magic, 4);
        if (std::string(magic, 4) != "MKBN")
        {
            throw std::runtime_error("Invalid magic header in dataset: " + path.string());
        }

        int version;
        file.read(reinterpret_cast<char *>(&version), 4);

        int num_cases;
        file.read(reinterpret_cast<char *>(&num_cases), 4);

        int dof;
        file.read(reinterpret_cast<char *>(&dof), 4);

        MixedDataset dataset;
        dataset.dof = dof;
        dataset.cases.reserve(num_cases);

        for (int i = 0; i < num_cases; ++i)
        {
            BenchmarkCase c;

            // Target Pos (3 doubles)
            Eigen::Vector3d pos;
            file.read(reinterpret_cast<char *>(pos.data()), 3 * 8);

            // Target Rot (9 doubles)
            Eigen::Matrix3d rot;
            file.read(reinterpret_cast<char *>(rot.data()), 9 * 8);

            Eigen::Isometry3d iso = Eigen::Isometry3d::Identity();
            iso.translate(pos);
            iso.rotate(Eigen::Quaterniond(rot));
            c.target_pose = Transform(iso);

            // Initial Guess (DOF doubles)
            c.initial_guess.resize(dof);
            file.read(reinterpret_cast<char *>(c.initial_guess.data()), dof * 8);

            // Ground Truth (DOF doubles)
            c.ground_truth.resize(dof);
            file.read(reinterpret_cast<char *>(c.ground_truth.data()), dof * 8);

            dataset.cases.push_back(c);
        }

        return dataset;
    }

    class MixedChainFixture : public benchmark::Fixture
    {
    public:
        void SetUp(const benchmark::State &state) override
        {
            // Assuming the argument passed to the benchmark is the DOF
            int dof = static_cast<int>(state.range(0));

            std::filesystem::path dataset_dir = std::filesystem::path("benchmarks/datasets");
            if (!std::filesystem::exists(dataset_dir))
            {
                // Try to resolve relative to executable if running from build dir
                dataset_dir = std::filesystem::current_path() / "benchmarks" / "datasets";
                if (!std::filesystem::exists(dataset_dir))
                {
                    // Fallback for when running from project root via ctest?
                    // Actually assume current working dir is project root or build/benchmarks
                    // Let's try absolute path or assume 'benchmarks/datasets' exists in CWD
                }
            }

            std::string dof_str = std::to_string(dof);
            std::filesystem::path urdf_path = dataset_dir / ("mixed_" + dof_str + "dof.urdf");
            std::filesystem::path data_path = dataset_dir / ("tier_b_" + dof_str + "dof.bin");

            if (!std::filesystem::exists(urdf_path) || !std::filesystem::exists(data_path))
            {
                // Skip later if files missing
                skip_ = true;
                return;
            }

            // Load Robot
            URDFParser parser;
            robot_ = parser.parseFile(urdf_path.string());
            if (!robot_)
            {
                throw std::runtime_error("Failed to parse URDF: " + urdf_path.string());
            }

            // Find End Link (last link for serial chain)
            // Our generator names links link_1...link_N
            end_link_ = "link_" + dof_str;
            base_link_ = "base_link";

            // Load Dataset
            dataset_ = loadDataset(data_path);

            // Configure Solver
            config_ = SolverConfig{};
            config_.tolerance = 1e-4;
            config_.max_iterations = 100;
            config_.enable_warm_start = true;
        }

        std::unique_ptr<SQPIKSolver> makeSolver() const
        {
            auto solver = std::make_unique<SQPIKSolver>(robot_, end_link_, base_link_);
            solver->setSolverConfig(config_);
            return solver;
        }

        bool skip_ = false;
        std::shared_ptr<Robot> robot_;
        std::string end_link_;
        std::string base_link_;
        MixedDataset dataset_;
        SolverConfig config_;
    };

} // namespace

BENCHMARK_DEFINE_F(MixedChainFixture, BM_MixedChainIK)(benchmark::State &state)
{
    if (skip_)
    {
        state.SkipWithError("Dataset or URDF not found. Run dataset generation first.");
        return;
    }

    auto solver = makeSolver();
    auto fk = std::make_shared<ForwardKinematics>(robot_, end_link_, base_link_);

    Eigen::VectorXd result(dataset_.dof);

    size_t solves = 0;
    size_t successes = 0;
    size_t total_iterations = 0;
    double total_pos_error = 0.0;
    double total_rot_error = 0.0;

    // Joint sensitivity metrics
    double total_revolute_error = 0.0;
    size_t revolute_count = 0;
    double total_prismatic_error = 0.0;
    size_t prismatic_count = 0;

    // Pre-classify joints
    std::vector<bool> is_prismatic;
    auto joints = robot_->getActuatedJoints();
    for (const auto &joint : joints)
    {
        is_prismatic.push_back(joint->getType() == JointType::Prismatic);
    }

    for (auto _ : state)
    {
        for (const auto &sample : dataset_.cases)
        {
            // Use initial guess from dataset
            // Note: dataset contains different variations (cold/warm), blindly iterating all
            // Ideally we should filter by type, but binary format mixed them.
            // For now, just solve all of them.

            auto status = solver->solve(sample.target_pose, sample.initial_guess, result);

            total_iterations += status.iterations;
            if (status.converged)
            {
                successes++;

                Transform achieved = fk->compute(result);
                double pos_err = (achieved.translation() - sample.target_pose.translation()).norm();
                double rot_err = Eigen::Quaterniond(sample.target_pose.rotation()).angularDistance(Eigen::Quaterniond(achieved.rotation()));

                total_pos_error += pos_err;
                total_rot_error += rot_err;

                // Calculate per-joint error vs ground truth
                Eigen::VectorXd joint_diff = (result - sample.ground_truth).cwiseAbs();
                for (Eigen::Index i = 0; i < joint_diff.size(); ++i)
                {
                    if (is_prismatic[static_cast<size_t>(i)])
                    {
                        total_prismatic_error += joint_diff[i];
                        prismatic_count++;
                    }
                    else
                    {
                        total_revolute_error += joint_diff[i];
                        revolute_count++;
                    }
                }
            }

            solves++;
        }
    }

    if (solves > 0)
    {
        state.counters["success_rate"] = 100.0 * successes / solves;
        state.counters["avg_iter"] = static_cast<double>(total_iterations) / solves;
        if (successes > 0)
        {
            state.counters["avg_pos_err_mm"] = (total_pos_error / successes) * 1000.0;
            state.counters["avg_rot_err_deg"] = (total_rot_error / successes) * 180.0 / std::numbers::pi;

            if (revolute_count > 0)
            {
                state.counters["avg_rev_err_deg"] = (total_revolute_error / revolute_count) * 180.0 / std::numbers::pi;
            }
            if (prismatic_count > 0)
            {
                state.counters["avg_pris_err_mm"] = (total_prismatic_error / prismatic_count) * 1000.0;
            }
        }
    }
}

BENCHMARK_DEFINE_F(MixedChainFixture, BM_MixedChainIK_ColdStart)(benchmark::State &state)
{
    if (skip_)
    {
        state.SkipWithError("Dataset or URDF not found.");
        return;
    }

    auto solver = makeSolver();
    // Disable warm start in solver config to ensure we test the algorithm's cold start capability
    // But wait, the "cold start" benchmark usually refers to the initial guess being "cold" (far/zero).
    // The solver's "enable_warm_start" usually means "use previous solution as guess".
    // Here we provide explicit initial guess from dataset.

    size_t solves = 0;
    size_t successes = 0;
    size_t total_iterations = 0;

    for (auto _ : state)
    {
        for (const auto &sample : dataset_.cases)
        {
            // Filter for Cold Start: Check if initial guess is Zero
            if (sample.initial_guess.norm() > 1e-6)
                continue;

            Eigen::VectorXd result(dataset_.dof);
            auto status = solver->solve(sample.target_pose, sample.initial_guess, result);

            total_iterations += status.iterations;
            if (status.converged)
                successes++;
            solves++;
        }
    }

    if (solves > 0)
    {
        state.counters["success_rate"] = 100.0 * successes / solves;
        state.counters["avg_iter"] = static_cast<double>(total_iterations) / solves;
    }
}

BENCHMARK_DEFINE_F(MixedChainFixture, BM_MixedChainIK_WarmStart)(benchmark::State &state)
{
    if (skip_)
    {
        state.SkipWithError("Dataset or URDF not found.");
        return;
    }

    auto solver = makeSolver();

    size_t solves = 0;
    size_t successes = 0;
    size_t total_iterations = 0;

    for (auto _ : state)
    {
        for (const auto &sample : dataset_.cases)
        {
            // Filter for Warm Start: Check if initial guess is close to GT
            // Assuming dataset has warm starts where guess = gt + noise
            // And cold starts are either zero or random (likely far)
            // Heuristic: if dist < 1.0 rad/m, assume warm?
            // Or better: check if it is NOT zero.
            // But random cold start is also not zero.
            // Let's assume warm start if error < 0.5 (since sigma=0.1, 3sigma=0.3)

            double dist = (sample.initial_guess - sample.ground_truth).norm();
            if (dist > 0.5)
                continue;

            Eigen::VectorXd result(dataset_.dof);
            auto status = solver->solve(sample.target_pose, sample.initial_guess, result);

            total_iterations += status.iterations;
            if (status.converged)
                successes++;
            solves++;
        }
    }

    if (solves > 0)
    {
        state.counters["success_rate"] = 100.0 * successes / solves;
        state.counters["avg_iter"] = static_cast<double>(total_iterations) / solves;
    }
}

// Register benchmarks for different DOFs
BENCHMARK_REGISTER_F(MixedChainFixture, BM_MixedChainIK)
    ->DenseRange(8, 20, 1)
    ->Unit(benchmark::kMillisecond);
BENCHMARK_REGISTER_F(MixedChainFixture, BM_MixedChainIK_ColdStart)
    ->DenseRange(8, 20, 1)
    ->Unit(benchmark::kMillisecond);
BENCHMARK_REGISTER_F(MixedChainFixture, BM_MixedChainIK_WarmStart)
    ->DenseRange(8, 20, 1)
    ->Unit(benchmark::kMillisecond);
