//
// Created by huangcanjia <huangcanjia0214@gmail.com> on 2026/9/1.
// Copyright (c) 2026 Graphics@XMU (https://graphics.xmu.edu.cn). All rights reserved.
//
#include "lbfgs_optimizer_geogram.h"
#include <geogram/numerics/optimizer.h>
#include <cassert>
#include <geolio/common/log.h>

namespace
{
    GEO::index_t iter;
}

namespace geolio
{
    LbfgsOptimizerGeogram* LbfgsOptimizerGeogram::instance_ = nullptr;

    LbfgsOptimizerGeogram::LbfgsOptimizerGeogram(
        ) {
        instance_ = this;
    }

    double LbfgsOptimizerGeogram::optimize(
        const unsigned int n,
        double* x
        ) {
        iter = 0;

        const GEO::Optimizer_var optimizer = GEO::Optimizer::create("HLBFGS");
        assert(!optimizer.is_null());
        optimizer->set_epsg(EPSG);
        optimizer->set_epsf(EPSF);
        optimizer->set_epsx(EPSX);
        optimizer->set_newiteration_callback(geogram_HLBFGS_newiteration_CB);
        optimizer->set_funcgrad_callback(geogram_HLBFGS_funcgrad_CB);
        optimizer->set_N(n);
        optimizer->set_M(INNER_ITERATIONS_NB);
        optimizer->set_max_iter(MAX_ITERATION);
        optimizer->optimize(x);

        double f;
        std::vector<double> g(n);
        funcgrad(n, x, f, g.data());
        return f;
    }

    void LbfgsOptimizerGeogram::geogram_HLBFGS_newiteration_CB(
        GEO::index_t n,
        const double* x,
        double f,
        const double* g,
        double gnorm
        ) {
        if (instance_->VERBOSE > 0 &&
            iter % instance_->VERBOSE == 0)
            LOG::DEBUG("L-BFGS opt iter: {:>4},\t f: {:>20.8f},\t gnorm: {:>20.8f}", iter, f, gnorm);
        ++iter;
    }

    void LbfgsOptimizerGeogram::geogram_HLBFGS_funcgrad_CB(
        const GEO::index_t n,
        double* x,
        double& f,
        double* g
        ) {
        instance_->funcgrad(n, x, f, g);
    }
}