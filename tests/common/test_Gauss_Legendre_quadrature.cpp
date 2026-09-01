//
// Created by huangcanjia <huangcanjia0214@gmail.com> on 2026/9/1.
// Copyright (c) 2026 Graphics@XMU (https://graphics.xmu.edu.cn). All rights reserved.
//

#include <ranges>
#include <gtest/gtest.h>
#include <geolio/common/log.h>
#include <geolio/common/Gauss_Legendre_quadrature.h>

namespace
{
    /* Calculate the analytic integral of the monomial x^k over [-1, 1] */

    /**
     * Formula: ∫_{-1}^{1} x^k dx = [1^(k+1) - (-1)^(k+1)] / (k+1)
     */
    double analytical_integral(
        const int k
        ) {
        if (k % 2 != 0) return 0.0;
        return 2.0 / (k + 1.0);
    }

    /**
     * x^k
     */
    double numerical_integral(
        const int k,
        const std::vector<std::pair<double, double>>& points_and_weights
        ) {
        double sum = 0.0;
        for (const auto& [p, w] : points_and_weights)
            sum += w * std::pow(p, k);
        return sum;
    }
}

namespace geolio::test
{
    class GaussLegendreQuadratureTest : public ::testing::TestWithParam<GEO::index_t> {
    public:
        void get_points_and_weights(
            const GEO::index_t order
            ) {
            geolio::get_Gauss_Legendre_quadrature(order, points_and_weights_);

            /* [0, 1] -> [-1, 1] */
            for (auto& [x, w] : points_and_weights_) {
                x = 2*x-1;
                w *= 2;
            }

            ASSERT_EQ(points_and_weights_.size(), order);
        }

        void eval_nodes_symmetry(
            ) const {
            for (GEO::index_t i = 0, i_end = points_and_weights_.size(); i < i_end; ++i) {
                EXPECT_GE(points_and_weights_[i].first, -1.0);
                EXPECT_LE(points_and_weights_[i].first, 1.0);
                EXPECT_NEAR(points_and_weights_[i].first, -points_and_weights_[i_end-1-i].first, 1e-14);
            }
        }

        void eval_weights_sum(
            ) {
            double sum = 0;
            for (const auto &w: points_and_weights_ | std::views::values)
                sum += w;
            EXPECT_NEAR(sum, 2, 1e-14);
        }

        void eval_k_polynomial(
            const GEO::index_t order
            ) const {
            for (int k = 0; k <= 2*order-1; ++k) {
                const double analytical = analytical_integral(k);
                const double numerical = numerical_integral(k, points_and_weights_);
                EXPECT_NEAR(analytical, numerical, 1e-14);
            }
        }

        std::vector<std::pair<double, double>> points_and_weights_;
    };

    TEST_P(GaussLegendreQuadratureTest, SupportsMultipleOrders) {
        const GEO::index_t n = GetParam();
        get_points_and_weights(n);

        eval_nodes_symmetry();
        eval_weights_sum();
        eval_k_polynomial(n);
    }

    INSTANTIATE_TEST_SUITE_P(OrderTest, GaussLegendreQuadratureTest,
        ::testing::Values(1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15));
}
