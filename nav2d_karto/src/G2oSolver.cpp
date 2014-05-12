#include "G2oSolver.h"

#include "g2o/core/block_solver.h"
#include "g2o/core/factory.h"
#include "g2o/core/optimization_algorithm_factory.h"
#include "g2o/core/optimization_algorithm_gauss_newton.h"
#include "g2o/solvers/csparse/linear_solver_csparse.h"
#include "g2o/solvers/cholmod/linear_solver_cholmod.h"
#include "g2o/types/slam2d/types_slam2d.h"

#include <ros/console.h>

typedef g2o::BlockSolver< g2o::BlockSolverTraits<-1, -1> > SlamBlockSolver;
typedef g2o::LinearSolverCSparse<SlamBlockSolver::PoseMatrixType> SlamCSparseLinearSolver;
typedef g2o::LinearSolverCholmod<SlamBlockSolver::PoseMatrixType> SlamCholmodLinearSolver;

G2oSolver::G2oSolver()
{
	// Initialize the SparseOptimizer
	SlamCholmodLinearSolver* linearSolver = new SlamCholmodLinearSolver();
	linearSolver->setBlockOrdering(false);
	SlamBlockSolver* blockSolver = new SlamBlockSolver(linearSolver);
	mOptimizer.setAlgorithm(new g2o::OptimizationAlgorithmGaussNewton(blockSolver));
}

G2oSolver::~G2oSolver()
{
	// destroy all the singletons
	g2o::Factory::destroy();
	g2o::OptimizationAlgorithmFactory::destroy();
	g2o::HyperGraphActionLibrary::destroy();
}

void G2oSolver::Clear()
{
	// freeing the graph memory
	mOptimizer.clear();
}

void G2oSolver::Compute()
{
	
}

void G2oSolver::AddNode(karto::Vertex<karto::LocalizedObjectPtr>* pVertex)
{
	karto::Pose2 odom = pVertex->GetVertexObject()->GetCorrectedPose(); 
	g2o::VertexSE2* poseVertex = new g2o::VertexSE2;
	poseVertex->setEstimate(g2o::SE2(odom.GetX(), odom.GetY(), odom.GetHeading()));
	poseVertex->setId(pVertex->GetVertexObject()->GetUniqueId());
	mOptimizer.addVertex(poseVertex);
}

void G2oSolver::AddConstraint(karto::Edge<karto::LocalizedObjectPtr>* pEdge)
{
	// Create a new edge
	g2o::EdgeSE2* odometry = new g2o::EdgeSE2;
	
	// Set source and target
	int sourceID = pEdge->GetSource()->GetVertexObject()->GetUniqueId();
	int targetID = pEdge->GetTarget()->GetVertexObject()->GetUniqueId();
	odometry->vertices()[0] = mOptimizer.vertex(sourceID);
	odometry->vertices()[1] = mOptimizer.vertex(targetID);
	if(odometry->vertices()[0] == NULL) ROS_ERROR("Source vertex with id %d does not exist!", sourceID);
	if(odometry->vertices()[0] == NULL) ROS_ERROR("Target vertex with id %d does not exist!", targetID);
	
	// Set the measurement (odometry distance between vertices)
	karto::LinkInfo* pLinkInfo = (karto::LinkInfo*)(pEdge->GetLabel());
	karto::Pose2 diff = pLinkInfo->GetPoseDifference();
	g2o::SE2 measurement(diff.GetX(), diff.GetY(), diff.GetHeading());
	odometry->setMeasurement(measurement);
	
	// Set the covariance of the measurement
	karto::Matrix3 precisionMatrix = pLinkInfo->GetCovariance().Inverse();
	Eigen::Matrix<double,3,3> info;
	info(0,0) = precisionMatrix(0,0);
	info(0,1) = info(1,0) = precisionMatrix(0,1);
	info(0,2) = info(2,0) = precisionMatrix(0,2);
	info(1,1) = precisionMatrix(1,1);
	info(1,2) = info(2,1) = precisionMatrix(1,2);
	info(2,2) = precisionMatrix(2,2);
	odometry->setInformation(info);
	
	// Add the constraint to the optimizer
	mOptimizer.addEdge(odometry);
}

const karto::ScanSolver::IdPoseVector& G2oSolver::GetCorrections() const
{
	return mCorrections;
}
