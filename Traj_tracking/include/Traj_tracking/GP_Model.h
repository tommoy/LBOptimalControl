#pragma once
#include <string>
#include <iostream>
#include <vector>
#include <Eigen/Dense>
#include "Kernel.h"
#include "GaussianProcess.h"
#include "Likelihood.h"
#include "GaussianProcessInference.h"

typedef gpr::GaussianProcess<double> GaussianProcessType;
typedef GaussianProcessType::VectorType VectorType;
typedef GaussianProcessType::MatrixType MatrixType;
typedef GaussianProcessType::DiagMatrixType DiagMatrixType;
typedef GaussianProcessType::VectorListType VectorListType;

typedef gpr::GaussianProcessInference<double> GaussianProcessInferenceType;
typedef GaussianProcessInferenceType::Pointer GaussianProcessInferenceTypePointer;
typedef gpr::GaussianLogLikelihood<double> LikelihoodType;
typedef LikelihoodType::Pointer LikelihoodTypePointer;
typedef gpr::GaussianExpKernel<double> GaussianExpKernelType;
typedef GaussianExpKernelType::Pointer GaussianExpKernelTypePointer;
typedef gpr::GaussianKernel<double> GaussianKernelType;
typedef GaussianKernelType::Pointer GaussianKernelTypePointer;

GaussianExpKernelTypePointer gk(new GaussianExpKernelType(1, 1));
GaussianProcessType::Pointer gp(new GaussianProcessType(gk));


class Residual_Part{
public:
    double Predection;

    GaussianProcessInferenceTypePointer Optimize();
    void Predict(GaussianProcessInferenceTypePointer gpi, double, double);
};

GaussianProcessInferenceTypePointer Residual_Part::Optimize() {
    unsigned n = 200;
    double noise = 0.1;

    static boost::minstd_rand randgen(static_cast<unsigned>(time(0)));
    static boost::normal_distribution<> dist(0, noise);
    static boost::variate_generator<boost::minstd_rand, boost::normal_distribution<> > r(randgen, dist);

    double start = -1;
    double stop = 1;
    for(unsigned i = 0; i < n; i++){
        VectorType x(2);
        x(0) = start + i*(stop-start)/n;
        x(1) = x(0);
        VectorType y(1);
        y(0) = exp(x(0)) + exp(x(1)) + r();
        gp->AddSample(x, y);
    }

    double step = 1e-1;
    unsigned iterations = 100;
    LikelihoodTypePointer lh(new LikelihoodType());
    GaussianProcessInferenceTypePointer gpi(new GaussianProcessInferenceType(lh, gp, step, iterations));
    bool exp_output = true;
    gpi->Optimize(false, exp_output);

    std::cout<<"Get Hyperparameter"<<std::endl;
    return gpi;
}

void Residual_Part::Predict(GaussianProcessInferenceTypePointer gpi, double feature1, double feature2) {
    GaussianProcessInferenceType::ParameterVectorType parameters = gpi->GetParameters();
    for(unsigned i = 0; i < parameters.size(); i++){
        parameters[i] = std::exp(parameters[i]);
    }

    GaussianKernelTypePointer k(new GaussianKernelType(1,1));
    k->SetParameters(parameters);
    gp->SetKernel(k);

    VectorType xx(2);
    xx(0) = feature1;
    xx(1) = feature2;
    Predection = gp->Predict(xx)[0];
    std::cout<<"Get the best estimation!"<<std::endl;
}

