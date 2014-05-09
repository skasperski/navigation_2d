#include "G2oSolver.h"

#include "g2o/core/block_solver.h"
#include "g2o/core/factory.h"
#include "g2o/core/optimization_algorithm_factory.h"
#include "g2o/core/optimization_algorithm_gauss_newton.h"

#include "g2o/solvers/csparse/linear_solver_csparse.h"
#include "g2o/solvers/cholmod/linear_solver_cholmod.h"

#include "g2o/types/slam2d/vertex_se2.h"
#include "g2o/types/slam2d/edge_se2.h"

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
	karto::Pose2 odom = pVertex->GetVertexObject()->GetOdometricPose(); 
	g2o::VertexSE2* poseVertex = new g2o::VertexSE2;
	poseVertex->setEstimate(g2o::SE2(odom.GetX(), odom.GetY(), odom.GetHeading()));
	mOptimizer.addVertex(poseVertex);
}

void G2oSolver::AddConstraint(karto::Edge<karto::LocalizedObjectPtr>* pEdge)
{
	
}

const karto::ScanSolver::IdPoseVector& G2oSolver::GetCorrections() const
{
	return mCorrections;
}
