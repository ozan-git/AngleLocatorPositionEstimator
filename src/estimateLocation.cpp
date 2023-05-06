#include <iostream>
#include <fstream>
#include <sstream>
#include <utility>
#include <vector>
#include <cmath>
#include <random>

struct Measurement {
    double x{}, y{}, rotZ{}, azimuth{};
};

struct Point {
    double x{}, y{};
};

using PositionError = std::pair<Point, double>;

class Locator {
public:
    explicit Locator(std::string input_path, std::string output_path)
            : input_path_(std::move(input_path)), output_path_(std::move(output_path)) {}

    void run() {
        std::ifstream input_file(input_path_);
        std::ofstream output_file(output_path_);

        std::string line;
        while (std::getline(input_file, line)) {
            std::vector<Measurement> measurements = parseMeasurements(line);

            if (measurements.size() < 2) {
                output_file << "fail, not enough measurements" << std::endl;
                continue;
            }

            PositionError position_error = EstimatePosition(measurements);
            output_file << position_error.first.x << ", " << position_error.first.y << ", " << position_error.second
                        << std::endl;
        }

        input_file.close();
        output_file.close();
    }

private:
    const std::string input_path_;
    const std::string output_path_;

    static double deg2rad(const double degree) noexcept {
        return degree * M_PI / 180.0;
    }

    [[nodiscard]] static std::vector<Measurement> parseMeasurements(const std::string &line) {
        const char delimiter = ';';
        char comma;
        std::vector<Measurement> measurements;
        std::istringstream inputStream(line);

        for (std::string measurementStr; std::getline(inputStream, measurementStr, delimiter);) {
            std::stringstream measurementStream(measurementStr);
            Measurement measurement{};
            measurementStream >> measurement.x >> comma >> measurement.y >> comma >> measurement.rotZ >> comma
                              >> measurement.azimuth;
            measurements.push_back(measurement);
        }

        return measurements;
    }

    static int countInliers(const Point &point, const std::vector<Measurement> &measurements, double &totalError) {
        const double maxError = 8.3;
        int numInliers = 0;
        totalError = 0;

        for (const auto &measurement: measurements) {
            const double angle = deg2rad(measurement.rotZ + measurement.azimuth);
            const double slope = std::tan(angle);
            const double yIntercept = measurement.y - slope * measurement.x;

            const double yExpected = slope * point.x + yIntercept;
            const double error = std::abs(point.y - yExpected);

            if (error < maxError) {
                ++numInliers;
                totalError += error;
            }
        }

        return numInliers;
    }

    static std::pair<std::vector<Measurement>::size_type, std::vector<Measurement>::size_type> get_random_indices(
            std::mt19937 &gen, std::uniform_int_distribution<std::vector<Measurement>::size_type> &dis) {
        return {dis(gen), dis(gen)};
    }

    static PositionError EstimatePosition(const std::vector<Measurement> &measurements) {
        const int ransacIterations = 10000;
        double bestError = std::numeric_limits<double>::max();

        int bestInliers = 0;
        Point bestPosition{0, 0};
        double bestErrorSum = bestError;

        std::random_device rd;
        std::mt19937 gen(rd());
        std::uniform_int_distribution<std::vector<Measurement>::size_type> dis(0, measurements.size() - 1);

        for (int iter = 0; iter < ransacIterations; ++iter) {
            auto [idx1, idx2] = get_random_indices(gen, dis);

            const Measurement &m1 = measurements[idx1];
            const Measurement &m2 = measurements[idx2];

            Point position = calculate_position(m1, m2);

            double errorSum;
            int inliers = countInliers(position, measurements, errorSum);

            if (inliers > bestInliers) {
                bestInliers = inliers;
                bestPosition = position;
                bestErrorSum = errorSum;
            }
        }
        bestError = (bestInliers == 0) ? bestError : bestErrorSum / bestInliers;
        return {bestPosition, bestError};
    }

    static Point calculate_position(const Measurement &measurement1, const Measurement &measurement2) {
        double angle1 = deg2rad(measurement1.rotZ + measurement1.azimuth);
        double angle2 = deg2rad(measurement2.rotZ + measurement2.azimuth);

        double slope1 = std::tan(angle1);
        double slope2 = std::tan(angle2);

        double intercept1 = measurement1.y - slope1 * measurement1.x;
        double intercept2 = measurement2.y - slope2 * measurement2.x;

        double x = (intercept2 - intercept1) / (slope1 - slope2);

        double y = slope1 * x + intercept1;

        return {x, y};
    }
};

int main() {
    std::string input_file(R"(C:\Users\orhan\CLionProjects\AngleLocatorPositionEstimator\src\test\q2_input[515].txt)");
    std::string output_file(R"(C:\Users\orhan\CLionProjects\AngleLocatorPositionEstimator\src\test\q2_output[515].txt)");

    Locator locator(input_file, output_file);
    locator.run();

    return 0;
}