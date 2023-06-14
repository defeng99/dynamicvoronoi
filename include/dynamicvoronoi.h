#ifndef _DYNAMICVORONOI_H_
#define _DYNAMICVORONOI_H_

#include <limits.h>
#include <stdio.h>
#include <stdlib.h>
#include <queue>

#include "bucketedqueue.h"

//! A DynamicVoronoi object computes and updates a distance map and Voronoi
//! diagram.
class DynamicVoronoi {
 public:
  DynamicVoronoi();
  ~DynamicVoronoi();

  //! Initialization with an empty map
  void initializeEmpty(int _sizeX, int _sizeY, bool initGridMap = true);
  //! Initialization with a given binary map (false==free, true==occupied)
  void initializeMap(int _sizeX, int _sizeY, bool** _gridMap);

  //! add an obstacle at the specified cell coordinate
  void occupyCell(int x, int y);
  //! remove an obstacle at the specified cell coordinate
  void clearCell(int x, int y);
  //! remove old dynamic obstacles and add the new ones
  void exchangeObstacles(std::vector<INTPOINT>& newObstacles);

  //! update distance map and Voronoi diagram to reflect the changes
  void update(bool updateRealDist = true);
  //! prune the Voronoi diagram
  void prune();
  //! prune the Voronoi diagram by globally revisiting all Voronoi nodes. Takes
  //! more time but gives a more sparsely pruned Voronoi graph. You need to call
  //! this after every call to udpate()
  void updateAlternativePrunedDiagram();
  //! retrieve the alternatively pruned diagram. see
  //! updateAlternativePrunedDiagram()
  int** alternativePrunedDiagram() { return alternativeDiagram_; };
  //! retrieve the number of neighbors that are Voronoi nodes (4-connected)
  int getNumVoronoiNeighborsAlternative(int x, int y);
  //! returns whether the specified cell is part of the alternatively pruned
  //! diagram. See updateAlternativePrunedDiagram.
  bool isVoronoiAlternative(int x, int y);

  //! returns the obstacle distance at the specified location
  float getDistance(int x, int y);
  //! returns whether the specified cell is part of the (pruned) Voronoi graph
  bool isVoronoi(int x, int y);
  //! checks whether the specficied location is occupied
  bool isOccupied(int x, int y);
  //! write the current distance map and voronoi diagram as ppm file
  void visualize(const char* filename = "result.ppm");

  //! returns the horizontal size of the workspace/map
  unsigned int getSizeX() { return sizeX_; }
  //! returns the vertical size of the workspace/map
  unsigned int getSizeY() { return sizeY_; }

 private:
  struct dataCell {
    float dist;     // euclidean distance
    char voronoi;   // State enum
    char queueing;  // QueueingState enum
    int obstX;      // the coordinate x of nearest obstacle
    int obstY;      // the coordinate y of nearest obstacle
    bool needsRaise;
    int sqdist;  // squared euclidean distance
  };

  // voronoi states of datacell
  typedef enum {
    voronoiKeep = -4,   // not need prune, just keep
    freeQueued = -3,    // tag to symbolize the added to pruneQueue_
    voronoiRetry = -2,  // retry to tell whether prune or not
    voronoiPrune = -1,  // need prune
    free = 0,           // default value
    occupied = 1        // obstacle occupied
  } State;

  // queue stats of datacell
  typedef enum {
    fwNotQueued = 1,  // default value
    fwQueued = 2,     // flag to avoid duplicate elements in open_
    fwProcessed = 3,  // Low wavefront activated to all 8 neighbors
    bwQueued = 4,     // flag to indicate once occupied but now freed
    // HERE:enum value all the same to fwNotQueued, they name the same meaning
    bwProcessed = 1  // Raise wavefront activated to all 8 neighbors
  } QueueingState;
  typedef enum { invalidObstData = SHRT_MAX / 2 } ObstDataState;
  typedef enum { pruned, keep, retry } markerMatchResult;

  // methods
  void setObstacle(int x, int y);
  void removeObstacle(int x, int y);
  inline void checkVoro(int x, int y, int nx, int ny, dataCell& c,
                        dataCell& nc);
  void recheckVoro();
  void commitAndColorize(bool updateRealDist = true);
  inline void reviveVoroNeighbors(int& x, int& y);

  inline bool isOccupied(int& x, int& y, dataCell& c);
  inline markerMatchResult markerMatch(int x, int y);
  inline bool markerMatchAlternative(int x, int y);
  inline int getVoronoiPruneValence(int x, int y);

  // queues

  BucketPrioQueue<INTPOINT> open_;
  std::queue<INTPOINT> pruneQueue_;
  BucketPrioQueue<INTPOINT> sortedPruneQueue_;

  std::vector<INTPOINT> removeList_;
  std::vector<INTPOINT> addList_;
  std::vector<INTPOINT> lastObstacles_;

  // maps
  int sizeY_;
  int sizeX_;
  // cell date formed as 2D array
  dataCell** data_;
  // original grid map indicating occupancy(1: occupied, 0: free)
  bool** gridMap_;
  // flag for clear gridMap_ cache
  bool allocatedGridMap_;

  // parameters
  int padding_;
  double doubleThreshold_;

  double sqrt2_;

  //  dataCell** getData(){ return data; }
  int** alternativeDiagram_;
};

#endif
