#include <benchmark/benchmark.h>
#include "urdfx/kinematics.h"
#include "urdfx/urdf_parser.h"
#include "urdfx/logging.h"
#include <filesystem>
#include <random>

using namespace urdfx;

namespace {

std::filesystem::path resolveUrdfPath() {
#ifdef URDFX_UR5_URDF_PATH
    std::filesystem::path configured{URDFX_UR5_URDF_PATH};
    if (std::filesystem::exists(configured)) {
        return configured;
    }
#endif
    auto fallback = std::filesystem::path(__FILE__).parent_path().parent_path() / "tests" / "ur5_urdf" / "ur5e.urdf";
    if (!std::filesystem::exists(fallback)) {
        throw std::runtime_error("Unable to locate UR5e URDF for benchmarks");
    }
    return fallback;
}

class JacobianBenchmarkFixture {
public:
    JacobianBenchmarkFixture() {
        setLogLevel(spdlog::level::err);

        URDFParser parser;
        auto urdf_path = resolveUrdfPath();
        robot_ = parser.parseFile(urdf_path.string());
        if (!robot_) {
            throw std::runtime_error("Failed to parse URDF");
        }
        
        std::string end_link = "tool0";
        if (!robot_->getLink(end_link)) end_link = "ee_link";
        if (!robot_->getLink(end_link)) end_link = "wrist_3_link";
        if (!robot_->getLink(end_link)) end_link = robot_->getRootLink();

        calc_ = std::make_unique<JacobianCalculator>(robot_, end_link);
        
        ForwardKinematics fk(robot_, end_link);
        dof_ = fk.getNumJoints();
        
        // Pre-generate random configurations
        std::mt19937 rng(42);
        std::uniform_real_distribution<double> dist(-3.14, 3.14);
        for (int i = 0; i < 1000; ++i) {
            Eigen::VectorXd q(dof_);
            for (size_t j = 0; j < dof_; ++j) {
                q[j] = dist(rng);
            }
            configs_.push_back(q);
        }
    }

    const JacobianCalculator& calc() const { return *calc_; }
    const Eigen::VectorXd& config(size_t i) const { return configs_[i % configs_.size()]; }

private:
    std::shared_ptr<Robot> robot_;
    std::unique_ptr<JacobianCalculator> calc_;
    size_t dof_;
    std::vector<Eigen::VectorXd> configs_;
};

const JacobianBenchmarkFixture& fixture() {
    static JacobianBenchmarkFixture instance;
    return instance;
}

} // namespace

static void BM_Jacobian(benchmark::State& state) {
    const auto& fx = fixture();
    size_t i = 0;
    for (auto _ : state) {
        Eigen::MatrixXd J = fx.calc().compute(fx.config(i++), JacobianType::Analytic);
        benchmark::DoNotOptimize(J.data());
    }
}

BENCHMARK(BM_Jacobian)->Unit(benchmark::kMicrosecond);
