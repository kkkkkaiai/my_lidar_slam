//
// Created by znfs on 2021/12/1.
//

#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/inference/Key.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/geometry/Rot3.h>
#include <gtsam/geometry/Pose3.h>

using namespace gtsam;

int main(){
    NonlinearFactorGraph graph;
    Values initialEstimate;
    gtsam::ISAM2Params gtsam_parameters;
    gtsam_parameters.relinearizeThreshold = 0.01;
    gtsam_parameters.relinearizeSkip = 1;
    ISAM2 isam(gtsam_parameters);

    initialEstimate.insert(0, Pose3(Rot3::RzRyRx(0, 0, 0), Point3(0,0,0)) );
    noiseModel::Diagonal::shared_ptr priorNoise = noiseModel::Diagonal::Variances(
            (Vector(6) << 1e-6, 1e-6, 1e-6, 1e-8, 1e-8, 1e-8).finished());
    graph.add(PriorFactor<Pose3>(0, Pose3(Rot3::RzRyRx(0, 0, 0), Point3(0, 0, 0)), priorNoise));

    isam.update(graph, initialEstimate);
    isam.update();
}