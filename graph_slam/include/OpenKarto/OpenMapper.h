/*
 * Copyright (C) 2006-2011, SRI International (R)
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#pragma once

#ifndef __OpenKarto_Mapper_h__
#define __OpenKarto_Mapper_h__

#ifdef USE_TBB
#include <tbb/mutex.h>
#include <tbb/parallel_for.h>
#include <tbb/blocked_range.h>
#include <tbb/blocked_range3d.h>
#endif

#include <OpenKarto/Event.h>
#include <OpenKarto/Pair.h>
#include <OpenKarto/Geometry.h>
#include <OpenKarto/StringHelper.h>
#include <OpenKarto/SensorData.h>
#include <OpenKarto/Grid.h>
#include <OpenKarto/GridIndexLookup.h>
#include <OpenKarto/Module.h>
#include <OpenKarto/OccupancyGrid.h>
#include <OpenKarto/TypeCasts.h>

namespace karto
{

  ///** \addtogroup OpenKarto */
  //@{

  /** 
   * Event arguments for the events in the Mapper
   */
  class MapperEventArguments : public EventArguments
  {
  public:
    /**
     * Mapper event arguments with the provided message
     * @param rMessage message
     */
    MapperEventArguments(const String& rMessage)
      : m_Message(rMessage)
    {
    }

    /**
     * Destructor
     */
    virtual ~MapperEventArguments()
    {
    }

  public:
    /**
     * Gets the event message
     * @return event message
     */
    const String& GetEventMessage() const
    {
      return m_Message;
    }

  private:
    String m_Message;
  };

#ifdef WIN32
  EXPORT_KARTO_EVENT(KARTO_EXPORT, MapperEventArguments)
#endif

  ////////////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////////////

  /**
   * Base class to tag edge labels
   */
  class EdgeLabel
  {
  public:
    /**
     * Default constructor
     */
    EdgeLabel()
    {
    }

    /**
     * Destructor
     */
    virtual ~EdgeLabel()
    {
    }
  }; // EdgeLabel

  ////////////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////////////

  // Contains the requisite information for the "spring"
  // that links two scans together--the pose difference and the uncertainty
  // (represented by a covariance matrix).
  class LinkInfo : public EdgeLabel
  {
  public:
    /**
     * Link between the given poses
     * @param rPose1 first pose
     * @param rPose2 second pose
     * @param rCovariance link uncertainty
     */
    LinkInfo(const Pose2& rPose1, const Pose2& rPose2, const Matrix3& rCovariance)
    {
      Update(rPose1, rPose2, rCovariance);
    }

    /**
     * Destructor
     */
    virtual ~LinkInfo()
    {
    }

  public:
    /**
     * Changes the link information to be the given parameters
     * @param rPose1 new first pose
     * @param rPose2 new second pose
     * @param rCovariance new link uncertainty
     */
    void Update(const Pose2& rPose1, const Pose2& rPose2, const Matrix3& rCovariance)
    {
      m_Pose1 = rPose1;
      m_Pose2 = rPose2;

      // transform second pose into the coordinate system of the first pose
      Transform transform(rPose1, Pose2());
      m_PoseDifference = transform.TransformPose(rPose2);

      // transform covariance into reference of first pose
      Matrix3 rotationMatrix;
      rotationMatrix.FromAxisAngle(0, 0, 1, -rPose1.GetHeading());

      m_Covariance = rotationMatrix * rCovariance * rotationMatrix.Transpose();
    }

    /**
     * Gets the first pose
     * @return first pose
     */
    inline const Pose2& GetPose1()
    {
      return m_Pose1;
    }

    /**
     * Gets the second pose
     * @return second pose
     */
    inline const Pose2& GetPose2()
    {
      return m_Pose2;
    }

    /**
     * Gets the pose difference
     * @return pose difference
     */
    inline const Pose2& GetPoseDifference()
    {
      return m_PoseDifference;
    }

    /**
     * Gets the link covariance
     * @return link covariance
     */
    inline const Matrix3& GetCovariance()
    {
      return m_Covariance;
    }

  private:
    Pose2 m_Pose1;
    Pose2 m_Pose2;
    Pose2 m_PoseDifference;
    Matrix3 m_Covariance;
  }; // LinkInfo

  ////////////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////////////

  template<typename T>
  class Edge;

  /**
   * Represents a object in a graph
   */
  template<typename T>
  class Vertex
  {
    friend class Edge<T>;

  public:
    /**
     * Constructs a vertex representing the given object
     * @param pObject object
     */
    Vertex(T pObject)
      : m_pObject(pObject)
    {
    }

    /**
     * Destructor
     */
    virtual ~Vertex()
    {
    }

    /**
     * Gets edges adjacent to this vertex
     * @return adjacent edges
     */
    inline const List<Edge<T>*>& GetEdges() const
    {
      return m_Edges;
    }

    /**
     * Gets the object associated with this vertex
     * @return the object
     */
    inline T GetVertexObject() const
    {
      return m_pObject;
    }

    /**
     * Gets a list of the vertices adjacent to this vertex
     * @return adjacent vertices
     */
    List<Vertex<T>*> GetAdjacentVertices() const
    {
      List<Vertex<T>*> vertices;

      karto_const_forEach(typename List<Edge<T>*>, &m_Edges)
      {
        Edge<T>* pEdge = *iter;

        // check both source and target because we have a undirected graph
        if (pEdge->GetSource() != this)
        {
          vertices.Add(pEdge->GetSource());
        }

        if (pEdge->GetTarget() != this)
        {
          vertices.Add(pEdge->GetTarget());
        }
      }

      return vertices;
    }

  private:
    /**
     * Adds the given edge to this vertex's edge list
     * @param pEdge edge to add
     */
    inline void AddEdge(Edge<T>* pEdge)
    {
      m_Edges.Add(pEdge);
    }

    T m_pObject;
    List<Edge<T>*> m_Edges;
  }; // Vertex<T>

  ////////////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////////////

  /**
   * Represents an edge in a graph
   */
  template<typename T>
  class Edge
  {
  public:
    /**
     * Constructs an edge from the source to target vertex
     * @param pSource source vertex
     * @param pTarget target vertex
     */
    Edge(Vertex<T>* pSource, Vertex<T>* pTarget)
      : m_pSource(pSource)
      , m_pTarget(pTarget)
      , m_pLabel(NULL)
    {
      m_pSource->AddEdge(this);
      m_pTarget->AddEdge(this);
    }

    /**
     * Destructor
     */
    virtual ~Edge()
    {
      m_pSource = NULL;
      m_pTarget = NULL;

      if (m_pLabel != NULL)
      {
        delete m_pLabel;
        m_pLabel = NULL;
      }
    }

  public:
    /**
     * Gets the source vertex
     * @return source vertex
     */
    inline Vertex<T>* GetSource() const
    {
      return m_pSource;
    }

    /**
     * Gets the target vertex
     * @return target vertex
     */
    inline Vertex<T>* GetTarget() const
    {
      return m_pTarget;
    }

    /**
     * Gets the link info
     * @return link info
     */
    inline EdgeLabel* GetLabel()
    {
      return m_pLabel;
    }

    /**
     * Sets the link payload
     * @param pLabel edge label
     */
    inline void SetLabel(EdgeLabel* pLabel)
    {
      m_pLabel = pLabel;
    }

  private:
    Vertex<T>* m_pSource;
    Vertex<T>* m_pTarget;
    EdgeLabel* m_pLabel;
  }; // class Edge<T>

  ////////////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////////////

  /**
   * Visitor class
   */
  template<typename T>
  class Visitor
  {
  public:
    /**
     * Applies the visitor to the vertex
     * @param pVertex
     * @return true if the visitor accepted the vertex, false otherwise
     */
    virtual kt_bool Visit(Vertex<T>* pVertex) = 0;
  }; // Visitor<T>

  ////////////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////////////

  template<typename T>
  class Graph;

  /**
   * Graph traversal algorithm
   */
  template<typename T>
  class GraphTraversal
  {
  public:
    /**
     * Traverser for the given graph
     * @param pGraph graph
     */
    GraphTraversal(Graph<T>* pGraph)
      : m_pGraph(pGraph)
    {
    }

    /**
     * Destructor
     */
    virtual ~GraphTraversal()
    {
    }

  public:
    /**
     * Traverses the graph starting at the given vertex and applying the visitor to all visited nodes
     * @param pStartVertex starting vertex
     * @param pVisitor visitor
     * @return list of vertices visited during traversal
     */
    virtual List<T> Traverse(Vertex<T>* pStartVertex, Visitor<T>* pVisitor) = 0;

  protected:
    /**
     * Graph being traversed
     */
    Graph<T>* m_pGraph;
  }; // GraphTraversal<T>

  ////////////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////////////

  /**
   * Graph
   */
  template<typename T>
  class Graph
  {
  public:
    /**
     * Type definition for list of vertices
     */
    typedef List<Vertex<T>*> VertexList;
    
    /**
     * Type definition for list of edges
     */
    typedef List<Edge<T>*> EdgeList;

  public:
    /**
     * Default constructor
     */
    Graph()
    {
    }

    /**
     * Destructor
     */
    virtual ~Graph()
    {
      Clear();
    }

  public:
    /**
     * Adds and indexes the given vertex into the map
     * @param pVertex vertex
     */
    inline void AddVertex(Vertex<T>* pVertex)
    {
      m_Vertices.Add(pVertex);
    }

    /**
     * Adds an edge to the graph
     * @param pEdge edge
     */
    inline void AddEdge(Edge<T>* pEdge)
    {
      m_Edges.Add(pEdge);
    }

    /**
     * Deletes the graph data
     */
    void Clear()
    {
      karto_const_forEach(typename VertexList, &m_Vertices)
      {
        // delete each vertex
        delete *iter;
      }

      m_Vertices.Clear();

      karto_const_forEach(typename EdgeList, &m_Edges)
      {
        // delete each edge
        delete *iter;
      }

      m_Edges.Clear();
    }

    /**
     * Gets the edges of this graph
     * @return graph edges
     */
    inline const EdgeList& GetEdges() const
    {
      return m_Edges;
    }

    /**
     * Gets the vertices of this graph
     * @return graph vertices
     */
    inline const VertexList& GetVertices() const
    {
      return m_Vertices;
    }

  protected:
    /**
     * Map of names to vector of vertices
     */
    VertexList m_Vertices;
    
    /**
     * Edges of this graph
     */
    EdgeList m_Edges;    
  }; // Graph<T>

  ////////////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////////////

  class OpenMapper;
  class ScanMatcher;

  /**
   * Graph for graph SLAM algorithm
   */
  class KARTO_EXPORT MapperGraph : public Graph<LocalizedObjectPtr>
  {
  public:
    /**
     * Graph for graph SLAM
     * @param pOpenMapper mapper
     * @param rangeThreshold range threshold
     */
    MapperGraph(OpenMapper* pOpenMapper, kt_double rangeThreshold);
    
    /**
     * Destructor
     */
    virtual ~MapperGraph();
    
  public:
    /**
     * Adds a vertex representing the given object to the graph
     * @param pObject object
     */
    void AddVertex(LocalizedObject* pObject);
    
    /**
     * Creates an edge between the source object and the target object if it
     * does not already exist; otherwise return the existing edge
     * @param pSourceObject source object
     * @param pTargetObject target object
     * @param rIsNewEdge set to true if the edge is new
     * @return edge between source and target vertices
     */
    Edge<LocalizedObjectPtr>* AddEdge(LocalizedObject* pSourceObject, LocalizedObject* pTargetObject, kt_bool& rIsNewEdge);

    /**
     * Links object to last scan
     * @param pObject object
     */
    void AddEdges(LocalizedObject* pObject);
    
    /**
     * Links scan to last scan and nearby chains of scans
     * @param pScan scan
     * @param rCovariance match uncertainty
     */
    void AddEdges(LocalizedLaserScan* pScan, const Matrix3& rCovariance);
    
    /**
     * Tries to close a loop using the given scan with the scans from the given sensor
     * @param pScan scan
     * @param rSensorName name of sensor
     * @return true if a loop was closed
     */
    kt_bool TryCloseLoop(LocalizedLaserScan* pScan, const Identifier& rSensorName);
    
    /**
     * Finds "nearby" (no further than given distance away) scans through graph links
     * @param pScan scan
     * @param maxDistance maximum distance
     * @return "nearby" scans that have a path of links to given scan
     */
    LocalizedLaserScanList FindNearLinkedScans(LocalizedLaserScan* pScan, kt_double maxDistance);

    /**
     * Finds scans that overlap the given scan (based on bounding boxes)
     * @param pScan scan
     * @return overlapping scans
     */
     LocalizedLaserScanList FindOverlappingScans(LocalizedLaserScan* pScan);
    
    /**
     * Gets the graph's scan matcher
     * @return scan matcher
     */    
    inline ScanMatcher* GetLoopScanMatcher() const
    {
      return m_pLoopScanMatcher;
    }

    /**
     * Links the chain of scans to the given scan by finding the closest scan in the chain to the given scan
     * @param rChain chain
     * @param pScan scan
     * @param rMean mean
     * @param rCovariance match uncertainty
     */
    void LinkChainToScan(const LocalizedLaserScanList& rChain, LocalizedLaserScan* pScan, const Pose2& rMean, const Matrix3& rCovariance);
    
  private:
    /**
     * Gets the vertex associated with the given scan
     * @param pScan scan
     * @return vertex of scan
     */
    inline Vertex<LocalizedObjectPtr>* GetVertex(LocalizedObject* pObject)
    {
      return m_Vertices[pObject->GetUniqueId()];
    }
        
    /**
     * Finds the closest scan in the vector to the given pose
     * @param rScans scan
     * @param rPose pose
     */
    LocalizedLaserScan* GetClosestScanToPose(const LocalizedLaserScanList& rScans, const Pose2& rPose) const;
            
    /**
     * Adds an edge between the two objects and labels the edge with the given mean and covariance
     * @param pFromObject from object
     * @param pToObject to object
     * @param rMean mean
     * @param rCovariance match uncertainty
     */
    void LinkObjects(LocalizedObject* pFromObject, LocalizedObject* pToObject, const Pose2& rMean, const Matrix3& rCovariance);
    
    /**
     * Finds nearby chains of scans and link them to scan if response is high enough
     * @param pScan scan
     * @param rMeans means
     * @param rCovariances match uncertainties
     */
    void LinkNearChains(LocalizedLaserScan* pScan, Pose2List& rMeans, List<Matrix3>& rCovariances);
    
    /**
     * Finds chains of scans that are close to given scan
     * @param pScan scan
     * @return chains of scans
     */
    List<LocalizedLaserScanList> FindNearChains(LocalizedLaserScan* pScan);
        
    /**
     * Compute mean of poses weighted by covariances
     * @param rMeans list of poses
     * @param rCovariances uncertainties
     * @return weighted mean
     */
    Pose2 ComputeWeightedMean(const Pose2List& rMeans, const List<Matrix3>& rCovariances) const;
    
    /**
     * Tries to find a chain of scan from the given sensor starting at the
     * given scan index that could possibly close a loop with the given scan
     * @param pScan scan
     * @param rSensorName name of sensor
     * @param rStartScanIndex start index
     * @return chain that can possibly close a loop with given scan
     */
    LocalizedLaserScanList FindPossibleLoopClosure(LocalizedLaserScan* pScan, const Identifier& rSensorName, kt_int32u& rStartScanIndex);
    
    /**
     * Optimizes scan poses
     */
    void CorrectPoses();
    
  private:
    /**
     * Mapper of this graph
     */
    OpenMapper* m_pOpenMapper;
    
    /**
     * Scan matcher for loop closures
     */
    ScanMatcher* m_pLoopScanMatcher;    
    
    /**
     * Traversal algorithm to find near linked scans
     */
    GraphTraversal<LocalizedObjectPtr>* m_pTraversal;    
  }; // MapperGraph

  ////////////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////////////
  
  /**
   * Graph optimization algorithm
   */
  class ScanSolver : public Referenced
  {
  public:
    /**
     * Vector of id-pose pairs
     */
    typedef List<Pair<kt_int32s, Pose2> > IdPoseVector;

    /**
     * Default constructor
     */
    ScanSolver()
    {
    }

  protected:
    //@cond EXCLUDE
    /**
     * Destructor
     */
    virtual ~ScanSolver()
    {
    }
    //@endcond

  public:
    /**
     * Solve!
     */
    virtual void Compute() = 0;

    /**
     * Gets corrected poses after optimization
     * @return optimized poses
     */
    virtual const IdPoseVector& GetCorrections() const = 0;

    /**
     * Adds a node to the solver
     */
    virtual void AddNode(Vertex<LocalizedObjectPtr>* /*pVertex*/)
    {
    }

    /**
     * Removes a node from the solver
     */
    virtual void RemoveNode(kt_int32s /*id*/)
    {
    }

    /**
     * Adds a constraint to the solver
     */
    virtual void AddConstraint(Edge<LocalizedObjectPtr>* /*pEdge*/)
    {
    }
    
    /**
     * Removes a constraint from the solver
     */
    virtual void RemoveConstraint(kt_int32s /*sourceId*/, kt_int32s /*targetId*/)
    {
    }
    
    /**
     * Resets the solver
     */
    virtual void Clear() {};
  }; // ScanSolver

  ////////////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////////////
  
  /**
   * Correlation grid used for scan matching
   */
  class CorrelationGrid : public Grid<kt_int8u>
  {    
  protected:
    //@cond EXCLUDE
    /**
     * Destructor
     */
    virtual ~CorrelationGrid()
    {
      delete [] m_pKernel;
    }
    //@endcond
    
  public:
    /**
     * Creates a correlation grid of given size and parameters
     * @param width width
     * @param height height
     * @param resolution resolution
     * @param smearDeviation amount to smear when adding scans to grid
     * @return correlation grid
     */
    static CorrelationGrid* CreateGrid(kt_int32s width, kt_int32s height, kt_double resolution, kt_double smearDeviation)
    {
      assert(resolution != 0.0);
      
      // +1 in case of roundoff
      kt_int32u borderSize = GetHalfKernelSize(smearDeviation, resolution) + 1;
      
      CorrelationGrid* pGrid = new CorrelationGrid(width, height, borderSize, resolution, smearDeviation);
      
      return pGrid;
    }
        
    /**
     * Gets the index into the data pointer of the given grid coordinate
     * @param rGrid grid coordinate
     * @param boundaryCheck whether to check the boundary of the grid
     * @return grid index
     */
    virtual kt_int32s GridIndex(const Vector2i& rGrid, kt_bool boundaryCheck = true) const
    {
      kt_int32s x = rGrid.GetX() + m_Roi.GetX();
      kt_int32s y = rGrid.GetY() + m_Roi.GetY();
      
      return Grid<kt_int8u>::GridIndex(Vector2i(x, y), boundaryCheck);
    }
    
    /**
     * Get the region of interest (ROI)
     * @return region of interest
     */
    inline const Rectangle2<kt_int32s>& GetROI() const
    {
      return m_Roi;
    }
    
    /**
     * Sets the region of interest (ROI)
     * @param roi location of the ROI
     */
    inline void SetROI(const Rectangle2<kt_int32s>& roi)
    {
      m_Roi = roi;
    }
    
    /**
     * Smears cell if the cell at the given point is marked as "occupied"
     * @param rGridPoint grid coordinate
     */
    inline void SmearPoint(const Vector2i& rGridPoint)
    {
      assert(m_pKernel != NULL);
      
      int gridIndex = GridIndex(rGridPoint);
      if (GetDataPointer()[gridIndex] != GridStates_Occupied)
      {
        return;
      }
      
      kt_int32s halfKernel = m_KernelSize / 2;
      
      // apply kernel
      for (kt_int32s j = -halfKernel; j <= halfKernel; j++)
      {
        kt_int8u* pGridAdr = GetDataPointer(Vector2i(rGridPoint.GetX(), rGridPoint.GetY() + j));
        
        kt_int32s kernelConstant = (halfKernel) + m_KernelSize * (j + halfKernel);
        
        // if a point is on the edge of the grid, there is no problem
        // with running over the edge of allowable memory, because
        // the grid has margins to compensate for the kernel size
        SmearInternal(halfKernel, kernelConstant, pGridAdr);
      }      
    }
    
  protected:
    /**
     * Constructs a correlation grid of given size and parameters
     * @param width width
     * @param height height
     * @param borderSize size of border
     * @param resolution resolution
     * @param smearDeviation amount to smear when adding scans to grid
     */
    CorrelationGrid(kt_int32u width, kt_int32u height, kt_int32u borderSize, kt_double resolution, kt_double smearDeviation)
      : Grid<kt_int8u>(width + borderSize * 2, height + borderSize * 2)
      , m_SmearDeviation(smearDeviation)
      , m_pKernel(NULL)
    {            
      GetCoordinateConverter()->SetScale(1.0 / resolution);

      // setup region of interest
      m_Roi = Rectangle2<kt_int32s>(borderSize, borderSize, width, height);
      
      // calculate kernel
      CalculateKernel();
    }
    
    /**
     * Sets up the kernel for grid smearing
     */
    virtual void CalculateKernel()
    {
      kt_double resolution = GetResolution();

      assert(resolution != 0.0);
      assert(m_SmearDeviation != 0.0);      
      
      // min and max distance deviation for smearing;
      // will smear for two standard deviations, so deviation must be at least 1/2 of the resolution
      const kt_double MIN_SMEAR_DISTANCE_DEVIATION = 0.5 * resolution;
      const kt_double MAX_SMEAR_DISTANCE_DEVIATION = 10 * resolution;
      
      // check if given value too small or too big
      if (!math::InRange(m_SmearDeviation, MIN_SMEAR_DISTANCE_DEVIATION, MAX_SMEAR_DISTANCE_DEVIATION))
      {
        StringBuilder error;
        error << "Mapper Error:  Smear deviation too small:  Must be between " << MIN_SMEAR_DISTANCE_DEVIATION << " and " << MAX_SMEAR_DISTANCE_DEVIATION;
        throw Exception(error.ToString());
      }
      
      // NOTE:  Currently assumes a two-dimensional kernel
      
      // +1 for center
      m_KernelSize = 2 * GetHalfKernelSize(m_SmearDeviation, resolution) + 1;
      
      // allocate kernel
      m_pKernel = new kt_int8u[m_KernelSize * m_KernelSize];
      if (m_pKernel == NULL)
      {
        throw Exception("Unable to allocate memory for kernel!");
      }
      
      // calculate kernel
      kt_int32s halfKernel = m_KernelSize / 2;
      for (kt_int32s i = -halfKernel; i <= halfKernel; i++)
      {
        for (kt_int32s j = -halfKernel; j <= halfKernel; j++)
        {
#ifdef WIN32
          kt_double distanceFromMean = _hypot(i * resolution, j * resolution);
#else
          kt_double distanceFromMean = hypot(i * resolution, j * resolution);
#endif
          kt_double z = exp(-0.5 * pow(distanceFromMean / m_SmearDeviation, 2));
          
          kt_int32u kernelValue = static_cast<kt_int32u>(math::Round(z * GridStates_Occupied));
          assert(math::IsUpTo(kernelValue, static_cast<kt_int32u>(255)));
          
          int kernelArrayIndex = (i + halfKernel) + m_KernelSize * (j + halfKernel);
          m_pKernel[kernelArrayIndex] = static_cast<kt_int8u>(kernelValue);
        }
      }
    }
    
    /**
     * Computes the kernel half-size based on the smear distance and the grid resolution.
     * Computes to two standard deviations to get 95% region and to reduce aliasing.
     * @param smearDeviation amount to smear when adding scans to grid
     * @param resolution resolution
     * @return kernel half-size based on the parameters
     */
    static kt_int32s GetHalfKernelSize(kt_double smearDeviation, kt_double resolution)
    {
      assert(resolution != 0.0);
      
      return static_cast<kt_int32s>(math::Round(2.0 * smearDeviation / resolution));
    }
    
  private:
    // \todo 1/5/2011: was this separated from SmearPoint in preparation for optimization?
    inline void SmearInternal(kt_int32s halfKernel, kt_int32s kernelConstant, kt_int8u* pGridAdr)
    {
      kt_int8u kernelValue;
      kt_int32s kernelArrayIndex;
      kt_int32s i;
      
      for (i = -halfKernel; i <= halfKernel; i++)
      {
        kernelArrayIndex = i + kernelConstant;
        
        kernelValue = m_pKernel[kernelArrayIndex];
        if (kernelValue > pGridAdr[i])
        {
          // kernel value is greater, so set it to kernel value
          pGridAdr[i] = kernelValue;
        }
      }
    }
    
    /**
     * The point readings are smeared by this value in X and Y to create a smoother response.
     * Default value is 0.03 meters.
     */
    kt_double m_SmearDeviation;
    
    // Size of one side of the kernel
    kt_int32s m_KernelSize;
    
    // Cached kernel for smearing
    kt_int8u* m_pKernel;
    
    // Region of interest
    Rectangle2<kt_int32s> m_Roi;
  }; // CorrelationGrid
  
  ////////////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////////////

  class ScanMatcherGridSet : public Referenced
  {
  public:
    ScanMatcherGridSet(CorrelationGrid* pCorrelationGrid,
      Grid<kt_double>* pSearchSpaceProbs,
      GridIndexLookup<kt_int8u>* pGridLookup)
      : m_pCorrelationGrid(pCorrelationGrid)
      , m_pSearchSpaceProbs(pSearchSpaceProbs)
      , m_pGridLookup(pGridLookup)
    {
    }

    virtual ~ScanMatcherGridSet()
    {
      delete m_pGridLookup;
    }

    SmartPointer<CorrelationGrid> m_pCorrelationGrid;
    SmartPointer<Grid<kt_double> > m_pSearchSpaceProbs;
    GridIndexLookup<kt_int8u>* m_pGridLookup;
  };

  ////////////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////////////
  
  class ScanMatcherGridSetBank;
  
  /**
   * Scan matcher
   */
  class KARTO_EXPORT ScanMatcher
  {
  public:
    /**
     * Destructor
     */
    virtual ~ScanMatcher();
    
  public:
    /**
     * Creates a scan matcher with the given parameters
     * @param pOpenMapper mapper
     * @param searchSize how much to search in each direction
     * @param resolution grid resolution
     * @param smearDeviation amount to smear when adding scans to grid
     * @param rangeThreshold cutoff for range readings
     * @return scan matcher
     */
    static ScanMatcher* Create(OpenMapper* pOpenMapper, kt_double searchSize, kt_double resolution, kt_double smearDeviation, kt_double rangeThreshold);
    
    /**
     * Matches given scan against set of scans
     * @param pScan scan being scan-matched
     * @param rBaseScans set of scans whose points will mark cells in grid as being occupied
     * @param rMean output parameter of mean (best pose) of match
     * @param rCovariance output parameter of covariance of match
     * @param doPenalize whether to penalize matches further from the search center
     * @param doRefineMatch whether to do finer-grained matching if coarse match is good
     * @return strength of response
     */
    kt_double MatchScan(LocalizedLaserScan* pScan, const LocalizedLaserScanList& rBaseScans, Pose2& rMean, Matrix3& rCovariance,
                        kt_bool doPenalize = true, kt_bool doRefineMatch = true);
    
    /**
     * Finds the best pose for the scan centering the search in the correlation grid
     * at the given pose and search in the space by the vector and angular offsets
     * in increments of the given resolutions
     * @param pScanMatcherGridSet set of grids used for scan matching
     * @param pScan scan to match against correlation grid
     * @param rSearchCenter the center of the search space
     * @param rSearchSpaceOffset searches poses in the area offset by this vector around search center
     * @param rSearchSpaceResolution how fine a granularity to search in the search space
     * @param searchAngleOffset searches poses in the angles offset by this angle around search center
     * @param searchAngleResolution how fine a granularity to search in the angular search space
     * @param doPenalize whether to penalize matches further from the search center
     * @param rMean output parameter of mean (best pose) of match
     * @param rCovariance output parameter of covariance of match
     * @param doingFineMatch whether to do a finer search after coarse search
     * @return strength of response
     */
    kt_double CorrelateScan(ScanMatcherGridSet* pScanMatcherGridSet, LocalizedLaserScan* pScan, const Pose2& rSearchCenter, const Vector2d& rSearchSpaceOffset, const Vector2d& rSearchSpaceResolution,
                            kt_double searchAngleOffset, kt_double searchAngleResolution,	kt_bool doPenalize, Pose2& rMean, Matrix3& rCovariance, kt_bool doingFineMatch);
    
    /**
     * Computes the positional covariance of the best pose
     * @param pSearchSpaceProbs probabilities of poses over search space
     * @param rBestPose best pose
     * @param bestResponse best response
     * @param rSearchCenter center of search space
     * @param rSearchSpaceOffset amount to search in each direction
     * @param rSearchSpaceResolution grid resolution of search
     * @param searchAngleResolution angular resolution of search
     * @param rCovariance output parameter covariance
     */
    static void ComputePositionalCovariance(Grid<kt_double>* pSearchSpaceProbs, const Pose2& rBestPose, kt_double bestResponse, const Pose2& rSearchCenter,
                                            const Vector2d& rSearchSpaceOffset, const Vector2d& rSearchSpaceResolution,
                                            kt_double searchAngleResolution, Matrix3& rCovariance);
    
    /**
     * Computes the angular covariance of the best pose
     * @param pScanMatcherGridSet set of grids used for scan matching
     * @param rBestPose best pose
     * @param bestResponse best response
     * @param rSearchCenter center of search space
     * @param searchAngleOffset amount to search in each angular direction
     * @param searchAngleResolution angular resolution of search
     * @param rCovariance output parameter covariance
     */
    static void ComputeAngularCovariance(ScanMatcherGridSet* pScanMatcherGridSet, const Pose2& rBestPose, kt_double bestResponse, const Pose2& rSearchCenter,
                                         kt_double searchAngleOffset, kt_double searchAngleResolution, Matrix3& rCovariance);

    /**
     * Gets the correlation grid data (for debugging).
     * NOTE: This only works in single-threaded mode.
     * @throws Exception if not in single-threaded mode.
     * @return correlation grid
     */
    CorrelationGrid* GetCorrelationGrid() const;
    
    /**
     * Gets the search grid data (for debugging).
     * NOTE: This only works in single-threaded mode.
     * @throws Exception if not in single-threaded mode.
     * @return search grid
     */
    Grid<kt_double>* GetSearchGrid() const;
    
    /**
     * Gets response at given position for given rotation (only look up valid points)
     * @param pScanMatcherGridSet set of grids used for scan matching
     * @param angleIndex index of angle
     * @param gridPositionIndex index of grid position
     * @return response
     */
    static kt_double GetResponse(ScanMatcherGridSet* pScanMatcherGridSet, kt_int32u angleIndex, kt_int32s gridPositionIndex);    
    
  private:
    /**
     * Marks cells where scans' points hit as being occupied
     * @param pCorrelationGrid correlation grid used for scan matching
     * @param rScans scans whose points will mark cells in grid as being occupied
     * @param viewPoint do not add points that belong to scans "opposite" the view point
     */
    static void AddScans(CorrelationGrid* pCorrelationGrid, const LocalizedLaserScanList& rScans, const Vector2d& rViewPoint);
    static void AddScansNew(CorrelationGrid* pCorrelationGrid, const LocalizedLaserScanList& rScans, const Vector2d& rViewPoint);
    
    /**
     * Marks cells where scans' points hit as being occupied.  Can smear points as they are added.
     * @param pCorrelationGrid correlation grid used for scan matching
     * @param pScan scan whose points will mark cells in grid as being occupied
     * @param viewPoint do not add points that belong to scans "opposite" the view point
     * @param doSmear whether the points will be smeared
     */
    static void AddScan(CorrelationGrid* pCorrelationGrid, LocalizedLaserScan* pScan, const Vector2d& rViewPoint, kt_bool doSmear = true);
    static void AddScanNew(CorrelationGrid* pCorrelationGrid, const Vector2dList& rValidPoints, kt_bool doSmear = true);
    
    /**
     * Computes which points in a scan are on the same side as the given viewpoint
     * @param pScan scan
     * @param rViewPoint viewpoint
     * @return points on the same side
     */
    static Vector2dList FindValidPoints(LocalizedLaserScan* pScan, const Vector2d& rViewPoint);
    
  protected:
    /**
     * Default constructor
     */
    ScanMatcher(OpenMapper* pOpenMapper)
      : m_pOpenMapper(pOpenMapper)
      , m_pScanMatcherGridSet(NULL)
      , m_pScanMatcherGridSetBank(NULL)
    {
    }
    
  private:
    OpenMapper* m_pOpenMapper;
    
    SmartPointer<ScanMatcherGridSet> m_pScanMatcherGridSet;
    ScanMatcherGridSetBank* m_pScanMatcherGridSetBank;
  }; // ScanMatcher
  
  ////////////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////////////

  class SensorDataManager;
  struct MapperSensorManagerPrivate;
  
  /**
   * Manages the sensors for the mapper
   */
  class KARTO_EXPORT MapperSensorManager
  {    
  public:
    /**
     * Sensor manager with the given parameters for the running buffer of each sensor
     * @param runningBufferMaximumSize maximum size for the running buffer used in scan-matching
     * @param runningBufferMaximumDistance maximum distance between first and last scan in the running buffer
     */
    MapperSensorManager(kt_int32u runningBufferMaximumSize, kt_double runningBufferMaximumDistance);
    
    /**
     * Destructor
     */
    virtual ~MapperSensorManager();
    
  public:
    /**
     * Registers a sensor (with given name); do
     * nothing if sensor already registered
     * @param rSensorName name of sensor
     */
    void RegisterSensor(const Identifier& rSensorName);
    
    /**
     * Gets object from given sensor with given ID
     * @param rSensorName name of sensor
     * @param stateId state id
     * @return localized object
     */
    LocalizedObject* GetLocalizedObject(const Identifier& rSensorName, kt_int32s stateId);
    
    /**
     * Gets names of all sensors
     * @return list of sensor names
     */
    List<Identifier> GetSensorNames();
    
    /**
     * Gets last scan of given sensor
     * @param rSensorName name of sensor
     * @return last localized laser scan of sensor
     */
    LocalizedLaserScan* GetLastScan(const Identifier& rSensorName);
    
    /**
     * Sets the last scan of sensor to the given scan
     * @param pScan scan
     */
    void SetLastScan(LocalizedLaserScan* pScan);
    
    /**
     * Resets the last scan of the given sensor
     * @param rSensorName name of sensor
     */
    void ClearLastScan(const Identifier& rSensorName);
    
    /**
     * Gets the object with the given unique id
     * @param uniqueId unique id
     * @return object with given id
     */
    LocalizedObject* GetLocalizedObject(kt_int32s uniqueId);
    
    /**
     * Adds localized object to object list of sensor that recorded the object
     * @param pObject object
     */
    void AddLocalizedObject(LocalizedObject* pObject);
    
    /**
     * Adds scan to running scans of sensor that recorded scan
     * @param pScan scan
     */
    void AddRunningScan(LocalizedLaserScan* pScan);
    
    /**
     * Gets scans of sensor
     * @param rSensorName name of sensor
     * @return scans of sensor
     */
    LocalizedLaserScanList& GetScans(const Identifier& rSensorName);
    
    /**
     * Gets the index of this scan in the sensor's list of scans; useful when
     * wanting to quickly find neighboring processed scans
     * @param pScan scan
     * @return index of scan into sensor's list of scans; -1 if not found
     */
    kt_int32s GetScanIndex(LocalizedLaserScan* pScan);
    
    /**
     * Gets running scans of sensor
     * @param rSensorName name of sensor
     * @return running scans of sensor
     */
    LocalizedLaserScanList& GetRunningScans(const Identifier& rSensorName);
    
    /**
     * Gets all scans of all sensors
     * @return all scans of all sensors
     */
    LocalizedLaserScanList GetAllScans();
    
    /**
     * Gets all objects of all sensors
     * @return all objects of all sensors
     */
    LocalizedObjectList GetAllObjects();

    /**
     * Deletes all scan managers of all sensors
     */
    void Clear();
    
  private:
    /**
     * Gets the sensor data manager for the given localized object
     * @return sensor data manager
     */
    inline SensorDataManager* GetSensorDataManager(LocalizedObject* pObject)
    {
      return GetSensorDataManager(pObject->GetSensorIdentifier());
    }
    
    /**
     * Gets the sensor data manager for the given id
     * @param rSensorName name of sensor
     * @return sensor data manager
     */
    SensorDataManager* GetSensorDataManager(const Identifier& rSensorName);
    
  private:
    MapperSensorManagerPrivate* m_pMapperSensorManagerPrivate;    
  }; // MapperSensorManager
  
  ////////////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////////////
  
  /**
   * Graph SLAM mapper. Creates a map given a set of localized laser scans.
   * The current Karto implementation is a high-performance
   * scan-matching algorithm that constructs a map from individual scans and corrects for
   * errors in the range and odometry data.
   *
   * The mapper supports multi-robot mapping from a common starting point.
   * NOTE: When using multiple sensors, the range thresholds of all sensors must be the same.
   * \todo Handle different range thresholds.
   *
   * The following parameters can be set on the Mapper.
   *
   *  \a UseScanMatching (Parameter\<kt_bool>)\n
   *     When set to true, the mapper will use a scan-matching algorithm. In most real-world situations
   *     this should be set to true so that the mapper algorithm can correct for noise and errors in
   *     odometry and scan data. In some simulator environments where the simulated scan and odometry
   *     data are very accurate, the scan-matching algorithm can produce worse results. In those cases,
   *     set to false to improve results.
   *     Default value is true.
   *
   *  \a UseScanBarycenter (Parameter\<kt_bool>)\n
   *     When set to true, the mapper will use the scan's barycenter (the centroid of the point readings)
   *     as the position of the scan when calculating distances between scans when determining if scans
   *     are far (or near) "enough" to each other.
   *     Default value is true.
   *
   *  \a MinimumTravelDistance (Parameter\<kt_double> - meters)\n
   *     Sets the minimum travel distance between scans.  If a new scan's position is more than this value
   *     from the previous scan, the mapper will use the data from the new scan. Otherwise, it will discard the
   *     new scan if it also does not meet the minimum change in heading requirement.
   *     For performance reasons, generally it is a good idea to only process scans if the robot
   *     has moved a reasonable amount.
   *     Default value is 0.2 (meters).
   *
   *  \a MinimumTravelHeading (Parameter\<kt_double> - radians)\n
   *     Sets the minimum heading change between scans. If a new scan's heading is more than this value
   *     from the previous scan, the mapper will use the data from the new scan.  Otherwise, it will discard the
   *     new scan if it also does not meet the minimum travel distance requirement.
   *     For performance reasons, generally it is a good idea to only process scans if the robot
   *     has moved a reasonable amount.
   *     Default value is the equivalent value in radians of 20 degrees.
   *
   *  \a ScanBufferSize (Parameter\<kt_int32u> - size)\n
   *     Scan buffer size is the length of the scan chain stored for scan matching.
   *     This value should be set to approximately "scanBufferMaximumScanDistance" / "minimumTravelDistance".
   *     The idea is to get an area approximately 20 meters long for scan matching.
   *     For example, if we add scans every minimumTravelDistance == 0.3 meters, then "scanBufferSize"
   *     should be 20 / 0.3 = 67.)
   *     Default value is 70.
   *
   *  \a ScanBufferMaximumScanDistance (Parameter\<kt_double> - meters)\n
   *     Scan buffer maximum scan distance is the maximum distance between the first and last scans
   *     in the scan chain stored for matching.
   *     Default value is 20.0.
   *
   *  \a UseResponseExpansion (Parameter\<kt_bool>)\n
   *     Whether to increase the search space if no good matches are initially found.  Default value is false.   
   *
   *  \a DistanceVariancePenalty (Parameter\<kt_double> - meters^2)\n
   *     Variance of penalty for deviating from odometry when scan-matching.  Use lower values if the robot's
   *     odometer is more accurate.
   *     Default value is 0.3.
   *
   *  \a MinimumDistancePenalty (Parameter\<kt_double>)\n
   *     Minimum value of the penalty multiplier so scores do not become too small.
   *     Default value is 0.5.
   *
   *  \a AngleVariancePenalty (Parameter\<kt_double> - radians^2)\n
   *     Variance of penalty for deviating from odometry when scan-matching.  Use lower values if the robot's
   *     odometer is more accurate.
   *     Default value is the equivalent value in radians of 20 degrees^2.
   *
   *  \a MinimumAnglePenalty (Parameter\<kt_double>)\n
   *     Minimum value of the penalty multiplier so scores do not become too small.  Angles are not very accurate
   *     and thus deviations from the odometer should not cause scores to drop precipitously.
   *     Default value is 0.9.
   *
   *  \a LinkMatchMinimumResponseFine (Parameter\<kt_double> - probability (>= 0.0, <= 1.0))\n
   *     Scans are linked only if the correlation response value is greater than this value.
   *     Default value is 0.6
   *
   *  \a LinkScanMaximumDistance (Parameter\<kt_double> - meters)\n
   *     Maximum distance between linked scans.  Scans that are farther apart will not be linked
   *     regardless of the correlation response value.  If this number is large, many scans
   *     will be considered "near".  This can cause the mapper to aggressively try to close loops
   *     with scans that aren't linked to the current scan even though they aren't that close.
   *     Default value is 5.0 meters.
   *
   *  \a CorrelationSearchSpaceDimension (Parameter\<kt_double> - meters)\n
   *     The size of the search grid used by the matcher when matching sequential scans.
   *     Default value is 0.3 meters which tells the matcher to use a 30cm x 30cm grid.
   *
   *  \a CorrelationSearchSpaceResolution (Parameter\<kt_double> - meters)\n
   *     The resolution (size of a grid cell) of the correlation grid.
   *     Default value is 0.01 meters.
   *
   *  \a CorrelationSearchSpaceSmearDeviation (Parameter\<kt_double> - meters)\n
   *     The point readings are smeared by this value in X and Y to create a smoother response.  Use higher values
   *     for environments that do not have crisp outlines (e.g., outdoor environments with bushes) or where the
   *     ground is uneven and rough (but flat overall).
   *     Default value is 0.03 meters.
   *
   *  \a CoarseSearchAngleOffset (Parameter\<kt_double> - radians)\n
   *     The range of angles to search in each direction from the robot's heading during a coarse search.
   *     Default value is the equivalent value in radians of 20 degrees.
   *
   *  \a FineSearchAngleOffset (Parameter\<kt_double> - radians)\n
   *     The range of angles to search in each direction from the robot's heading during a finer search.
   *     Default value is the equivalent value in radians of 0.2 degrees.
   *
   *  \a CoarseAngleResolution (Parameter\<kt_double> - meters)\n
   *     Resolution of angles to search during a coarse search
   *     Default value is the equivalent value in radians of 2 degrees.
   *
   *  \a LoopSearchSpaceDimension (Parameter\<kt_double> - meters)\n
   *     The size of the search grid used by the matcher when detecting loop closures.
   *     Default value is 8.0 meters which tells the matcher to use a 8m x 8m grid.
   *
   *  \a LoopSearchSpaceResolution (Parameter\<kt_double> - meters)\n
   *     The resolution (size of a grid cell) of the correlation grid used by the matcher to
   *     determine a possible loop closure.
   *     Default value is 0.05 meters.
   *
   *  \a LoopSearchSpaceSmearDeviation (Parameter\<kt_double> - meters)\n
   *     The point readings are smeared by this value in X and Y to create a smoother response.
   *     Used by the matcher to determine a possible loop closure match.
   *     Default value is 0.03 meters.
   *
   *  \a LoopSearchMaximumDistance (Parameter\<kt_double> - meters)\n
   *     Scans less than this distance from the current position will be considered for a match
   *     in loop closure.
   *     Default value is 4.0 meters.
   *
   *  \a LoopMatchMinimumChainSize (Parameter\<kt_int32u>)\n
   *     When the loop closure detection finds a candidate it must be part of a large
   *     set of linked scans. If the chain of scans is less than this value, we do not attempt
   *     to close the loop.
   *     Default value is 10.
   *
   *  \a LoopMatchMaximumVarianceCoarse (Parameter\<kt_double>)\n
   *     The co-variance values for a possible loop closure have to be less than this value
   *     to consider a viable solution. This applies to the coarse search.
   *     Default value is 0.16.
   *
   *  \a LoopMatchMinimumResponseCoarse (Parameter\<kt_double> - probability (>= 0.0, <= 1.0))\n
   *     If response is larger then this, then initiate loop closure search at the coarse resolution.
   *     Default value is 0.7.
   *
   *  \a LoopMatchMinimumResponseFine (Parameter\<kt_double> - probability (>= 0.0, <= 1.0))\n
   *     If response is larger then this, then initiate loop closure search at the fine resolution.
   *     Default value is 0.7.
   */
  class KARTO_EXPORT OpenMapper : public Module
  {
    friend class MapperGraph;
    friend class ScanMatcher;

  public:
    /**
     * Default constructor
     * @param multiThreaded
     */
    OpenMapper(kt_bool multiThreaded = true);

    /**
     * Create a Mapper with a name
     * @param pName mapper name
     * @param multiThreaded
     */
    OpenMapper(const char* pName, kt_bool multiThreaded = true);

  public:
    /**
     * Event for generic mapper message. 
     * The mapper currently posts Message events when calculating possible loop 
     * closures and when rejecting a possible loop close.
     */
    BasicEvent<MapperEventArguments> Message;

    /**
     * Event fires before a loop closure is about to happen.
     * Calculating loop closure can be a CPU intensive task and the Mapper is blocking all Process calls.
     * If mapper is running on "real" robot please either pause the robot or stop adding scans to the mapper. 
     */
    BasicEvent<MapperEventArguments> PreLoopClosed;

    /**
     * Event fires after a loop was closed. 
     */
    BasicEvent<MapperEventArguments> PostLoopClosed;
    
    /**
     * Event fires when the scans have been changed. 
     * Scans are changed after a loop closure and after any translation, rotation or transform.
     */    
    BasicEvent<EventArguments> ScansUpdated;

  protected:
    //@cond EXCLUDE
    /**
     * Destructor
     */
    virtual ~OpenMapper();
    //@endcond
    
  public:
    /** 
     * Is mapper multi threaded
     * @return true if multi threaded, false otherwise
     */
    inline kt_bool IsMultiThreaded()
    {
      return m_MultiThreaded;
    }

    /**
     * Allocates memory needed for mapping. Please Reset() the mapper if range threshold on the laser range 
     * finder is changed after the mapper is initialized.
     * @param rangeThreshold range threshold
     */
    void Initialize(kt_double rangeThreshold);
    
    /**
     * Resets the mapper.
     * Deallocates memory allocated in Initialize()
     */
    void Reset();

    /**
     * Processes an object.  If the object is a scan, the scan must be identified with a range finder sensor.
     * Once added to a map, the corrected pose information in the localized object will be updated to the
     * correct pose as determined by the mapper.
     * @param pObject object
     * @return true if the object was processed successfully, false otherwise
     */
    virtual kt_bool Process(Object* pObject);
    
    /**
     * Returns all processed scans added to the mapper.
     * NOTE: The returned scans have their corrected pose updated.
     * @return list of scans received and processed by the mapper. If no scans have been processed,
     * returns an empty list.
     */
    const LocalizedLaserScanList GetAllProcessedScans() const;

    /**
     * Returns all processed objects added to the mapper.
     * NOTE: The returned objects have their corrected pose updated.
     * @return list of objects received and processed by the mapper. If no objects have been processed,
     * returns an empty list.
     */
    const LocalizedObjectList GetAllProcessedObjects() const;

    /**
     * Gets scan optimizer used by mapper when closing the loop
     * @return scan solver
     */
    ScanSolver* GetScanSolver() const;

    /**
     * Sets scan optimizer used by mapper when closing the loop
     * @param pSolver solver
     */
    void SetScanSolver(ScanSolver* pSolver);

    /**
     * Gets scan link graph
     * @return graph
     */
    virtual MapperGraph* GetGraph() const;

    /**
     * Gets the sequential scan matcher
     * @return sequential scan matcher
     */
    ScanMatcher* GetSequentialScanMatcher() const;

    /**
     * Gets the loop scan matcher
     * @return loop scan matcher
     */
    ScanMatcher* GetLoopScanMatcher() const;
    
    /**
     * Gets the sensor manager
     * @return sensor manager
     */
    inline MapperSensorManager* GetMapperSensorManager() const
    {
      return m_pMapperSensorManager;
    }
    
    /**
     * Tries to close a loop using the given scan with the scans from the given sensor
     * @param pScan scan
     * @param rSensorName name of sensor
     * @return true if a loop was closed
     */
    inline kt_bool TryCloseLoop(LocalizedLaserScan* pScan, const Identifier& rSensorName)
    {
      return m_pGraph->TryCloseLoop(pScan, rSensorName);
    }
    
  protected:
    /**
     * Hook called after scan matching and before trying loop closure
     * @param pScan scan that was scan-matched
     */
    virtual void ScanMatched(LocalizedLaserScan* pScan) {};
    
    /**
     * Hook called after scan matching and closing any loops
     * @param pScan scan that was scan-matched
     */
    virtual void ScanMatchingEnd(LocalizedLaserScan* pScan) {};

  private:
    void InitializeParameters();

    /**
     * Tests if the scan is "sufficiently far" from the last scan added.
     * @param pScan scan to be checked
     * @param pLastScan last scan added to mapper
     * @return true if the scan is "sufficiently far" from the last scan added or
     * the scan is the first scan to be added
     */
    kt_bool HasMovedEnough(LocalizedLaserScan* pScan, LocalizedLaserScan* pLastScan) const;

  public:
    /////////////////////////////////////////////
    // fire information for listeners!!

    /**
     * Returns initialized flag for mapper
     */
    kt_bool IsInitialized()
    {
      return m_Initialized;
    }

  private:
    // restrict the following functions
    OpenMapper(const OpenMapper&);
    const OpenMapper& operator=(const OpenMapper&);

  protected:
    /**
     * Solver used to optimize scan poses on pose graph
     */
    SmartPointer<ScanSolver> m_pScanSolver;

  private:
    kt_bool m_Initialized;
    kt_bool m_MultiThreaded;

    ScanMatcher* m_pSequentialScanMatcher;

    MapperSensorManager* m_pMapperSensorManager;

    MapperGraph* m_pGraph;

    /////////////////////////////////////////////////////////////////////////////////////////
    // Parameters
    // NOTE: Maintain the descriptions to these parameters in the comment above the class!
    /////////////////////////////////////////////////////////////////////////////////////////

    Parameter<kt_bool>* m_pUseScanMatching;
    Parameter<kt_bool>* m_pUseScanBarycenter;    
    Parameter<kt_double>* m_pMinimumTravelDistance;
    Parameter<kt_double>* m_pMinimumTravelHeading;

    //////////////////////////////////////////////////////////////////////////////
    // scan matcher parameters
    
    Parameter<kt_int32u>* m_pScanBufferSize;
    Parameter<kt_double>* m_pScanBufferMaximumScanDistance;
    Parameter<kt_bool>* m_pUseResponseExpansion;

    Parameter<kt_double>* m_pDistanceVariancePenalty;
    Parameter<kt_double>* m_pMinimumDistancePenalty;
    Parameter<kt_double>* m_pAngleVariancePenalty;
    Parameter<kt_double>* m_pMinimumAnglePenalty;
    
    Parameter<kt_double>* m_pLinkMatchMinimumResponseFine;
    Parameter<kt_double>* m_pLinkScanMaximumDistance;

    //////////////////////////////////////////////////////////////////////////////
    // correlation parameters

    Parameter<kt_double>* m_pCorrelationSearchSpaceDimension;
    Parameter<kt_double>* m_pCorrelationSearchSpaceResolution;
    Parameter<kt_double>* m_pCorrelationSearchSpaceSmearDeviation;
    Parameter<kt_double>* m_pCoarseSearchAngleOffset;
    Parameter<kt_double>* m_pFineSearchAngleOffset;
    Parameter<kt_double>* m_pCoarseAngleResolution;

    //////////////////////////////////////////////////////////////////////////////
    // loop correlation parameters

    Parameter<kt_double>* m_pLoopSearchSpaceDimension;
    Parameter<kt_double>* m_pLoopSearchSpaceResolution;
    Parameter<kt_double>* m_pLoopSearchSpaceSmearDeviation;

    //////////////////////////////////////////////////////////////////////////////
    // loop detection parameters
    
    Parameter<kt_double>* m_pLoopSearchMaximumDistance;    
    Parameter<kt_int32u>* m_pLoopMatchMinimumChainSize;    
    Parameter<kt_double>* m_pLoopMatchMaximumVarianceCoarse;    
    Parameter<kt_double>* m_pLoopMatchMinimumResponseCoarse;    
    Parameter<kt_double>* m_pLoopMatchMinimumResponseFine;
  };

  //@}
}

#endif // __OpenKarto_Mapper_h__
