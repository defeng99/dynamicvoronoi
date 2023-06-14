#include "dynamicvoronoi.h"

#include <math.h>
#include <iostream>

DynamicVoronoi::DynamicVoronoi() {
  sqrt2_ = sqrt(2.0);
  data_ = NULL;
  gridMap_ = NULL;
  alternativeDiagram_ = NULL;
  allocatedGridMap_ = false;
}

DynamicVoronoi::~DynamicVoronoi() {
  if (data_) {
    for (int x = 0; x < sizeX_; x++) delete[] data_[x];
    delete[] data_;
  }
  // HERE: 利用allocatedGridMap_管理gridMap_
  if (allocatedGridMap_ && gridMap_) {
    for (int x = 0; x < sizeX_; x++) delete[] gridMap_[x];
    delete[] gridMap_;
  }
}

void DynamicVoronoi::initializeEmpty(int _sizeX, int _sizeY, bool initGridMap) {
  //先清空历史数据
  if (data_) {
    for (int x = 0; x < sizeX_; x++) delete[] data_[x];
    delete[] data_;
    data_ = NULL;
  }
  if (alternativeDiagram_) {
    for (int x = 0; x < sizeX_; x++) delete[] alternativeDiagram_[x];
    delete[] alternativeDiagram_;
    alternativeDiagram_ = NULL;
  }
  // HERE:偶尔只需清空结果文件，而不是输入文件：gridMap_
  if (initGridMap) {
    if (allocatedGridMap_ && gridMap_) {
      for (int x = 0; x < sizeX_; x++) delete[] gridMap_[x];
      delete[] gridMap_;
      gridMap_ = NULL;
      allocatedGridMap_ = false;
    }
  }

  //为数组分配内存
  sizeX_ = _sizeX;
  sizeY_ = _sizeY;
  data_ = new dataCell*[sizeX_];
  for (int x = 0; x < sizeX_; x++) data_[x] = new dataCell[sizeY_];

  if (initGridMap) {
    gridMap_ = new bool*[sizeX_];
    for (int x = 0; x < sizeX_; x++) gridMap_[x] = new bool[sizeY_];
    allocatedGridMap_ = true;
  }

  // HERE: dataCell的默认值：
  dataCell c;
  c.dist = INFINITY;
  c.sqdist = INT_MAX;
  c.obstX = invalidObstData;
  c.obstY = invalidObstData;
  c.voronoi = free;
  c.queueing = fwNotQueued;
  c.needsRaise = false;

  //为数组填充初始化数据
  for (int x = 0; x < sizeX_; x++)
    for (int y = 0; y < sizeY_; y++) data_[x][y] = c;

  if (initGridMap) {
    for (int x = 0; x < sizeX_; x++)
      for (int y = 0; y < sizeY_; y++) gridMap_[x][y] = 0;
  }
}

//输入二值地图gridmap，根据元素是否被占用，更新data_
void DynamicVoronoi::initializeMap(int _sizeX, int _sizeY, bool** _gridMap) {
  // HERE:initGridMap == false，并没有释放原先gridMap_分配的内存，容易内存泄露
  // TODO:因此initializeMap()和initializeEmpty()仅作为初始化调用一次，需要进行调用次数保护
  gridMap_ = _gridMap;
  initializeEmpty(_sizeX, _sizeY, false);

  for (int x = 0; x < sizeX_; x++) {
    for (int y = 0; y < sizeY_; y++) {
      //只需尝试更新被占用的位置
      if (gridMap_[x][y]) {
        dataCell c = data_[x][y];
        // 很明显，初始化时，data_内并没有生成占用关系，因而如下条件必然满足！
        if (!isOccupied(x, y, c)) {
          //如果8邻接datacell全被占用，属于内部障碍物，令isSurrounded = true
          bool isSurrounded = true;
          // TODO:邻接点检测顺序是否可以优化？
          for (int dx = -1; dx <= 1; dx++) {
            if (!isSurrounded) {
              //再次跳出此层for循环
              break;
            }
            int nx = x + dx;
            if (nx <= 0 || nx >= sizeX_ - 1) continue;
            for (int dy = -1; dy <= 1; dy++) {
              if (dx == 0 && dy == 0) continue;
              int ny = y + dy;
              if (ny <= 0 || ny >= sizeY_ - 1) continue;
              //如果8邻接元素有任意一个未被占用，即属于边界障碍物！
              if (!gridMap_[nx][ny]) {
                isSurrounded = false;
                break;
              }
            }
          }
          //对内部障碍物栅格，进行普通赋值，让其不可能进入open_
          if (isSurrounded) {
            c.obstX = x;
            c.obstY = y;
            c.sqdist = 0;
            c.dist = 0;
            c.voronoi = occupied;
            c.queueing = fwProcessed;
            data_[x][y] = c;
          } else {
            //障碍物边界栅格需要进入open_队列，形成wavefront！
            setObstacle(x, y);
          }
        }
        //而且，这是初始化，data_原先不存在占用情况。如果不是初始化，那么有可能存在：
        //原先被占用，但出现 边界占用 <---> 非边界占用 的状态变更
      }
    }
  }
}

//要同时更新gridmap和data_
void DynamicVoronoi::occupyCell(int x, int y) {
  gridMap_[x][y] = 1;  //更新gridmap
  setObstacle(x, y);
}

//要同时更新gridmap和data_
void DynamicVoronoi::clearCell(int x, int y) {
  gridMap_[x][y] = 0;  //更新gridmap
  removeObstacle(x, y);
}

//只更新data_内的占用情况
void DynamicVoronoi::setObstacle(int x, int y) {
  dataCell c = data_[x][y];
  if (isOccupied(x, y, c)) {  //避免重复操作
    return;
  }

  addList_.push_back(INTPOINT(x, y));  //加入addList_
  // HERE:此处仅仅更新了占用情况，其他全部默认值！
  c.obstX = x;
  c.obstY = y;
  data_[x][y] = c;
}

//只更新data_内的占用情况
void DynamicVoronoi::removeObstacle(int x, int y) {
  dataCell c = data_[x][y];
  if (!isOccupied(x, y, c)) {  //避免重复操作
    return;
  }

  removeList_.push_back(INTPOINT(x, y));  //加入removeList_
  c.obstX = invalidObstData;
  c.obstY = invalidObstData;
  // HERE:额外更新排队状态,使其能逃过fwProcessed状态检查
  c.queueing = bwQueued;
  data_[x][y] = c;
}

// REVIEW:如果多次调用该方法，逐个地添加或移除障碍物，需要提前将gridMap_更新到最终状态？！
//用新的障碍物信息替换旧的障碍物信息
// points代表新障碍物，lastObstacles_代表旧障碍物
void DynamicVoronoi::exchangeObstacles(std::vector<INTPOINT>& points) {
  // HERE:若移除和添加的障碍物存在交集部分，
  //需要严格遵守先移除，后增加的处理顺序，这样才能保证占用关系处理正确！
  for (unsigned int i = 0; i < lastObstacles_.size(); i++) {
    int x = lastObstacles_[i].x;
    int y = lastObstacles_[i].y;
    // HERE:首先gridMap_已经更新了最新的障碍物占用情况，
    // 因此从gridMap_上可以判断处移除和添加的重叠部分！
    if (gridMap_[x][y]) {
      continue;
    }
    removeObstacle(x, y);
  }
  lastObstacles_.clear();

  for (unsigned int i = 0; i < points.size(); i++) {
    int x = points[i].x;
    int y = points[i].y;
    // HERE:同上，重叠部分不需要处理！
    if (gridMap_[x][y]) {
      continue;
    }
    //并不区分是否在边界，以及是否改变周边的占用性质（边界 <---> 非边界）
    setObstacle(x, y);
    lastObstacles_.push_back(points[i]);
  }
}

void DynamicVoronoi::update(bool updateRealDist) {
  //将发生状态变化（占用<-->不占用）的元素加入open_优先队列
  commitAndColorize(updateRealDist);

  while (!open_.empty()) {
    INTPOINT p = open_.pop();
    int x = p.x;
    int y = p.y;
    dataCell c = data_[x][y];

    // HERE:安全检查，将上次结果内的无需更新的栅格给保护性剔除掉
    if (c.queueing == fwProcessed) {
      continue;
    }

    if (c.needsRaise) {
      // RAISE过程：从被删除栅格为起始，逐步向外激发RAISE波，直到抵达无需更新状态的边界！
      //从8邻接栅格开始形成wavefront
      for (int dx = -1; dx <= 1; dx++) {
        int nx = x + dx;
        if (nx <= 0 || nx >= sizeX_ - 1) continue;
        for (int dy = -1; dy <= 1; dy++) {
          if (dx == 0 && dy == 0) continue;
          int ny = y + dy;
          if (ny <= 0 || ny >= sizeY_ - 1) continue;
          dataCell nc = data_[nx][ny];
          // HERE:邻接栅格的最近障碍物不是非法值，并且不在此层Raise波上！此处条件即限定住Raise波始终向外，不会重复访问！
          if (nc.obstX != invalidObstData && !nc.needsRaise) {
            // HERE:邻接栅格的最近障碍物位于删除部分，因此需要激发Raise波，并将sqdist设置成INT_MAX便于之后Low波中更新！
            if (!isOccupied(nc.obstX, nc.obstY, data_[nc.obstX][nc.obstY])) {
              // HERE:之前的最近距离与删除部分的远近关系等同，通过将其加入队列，能控制Raise波的扩展方向具有单调向外，不会错误地跳过更内部的未访问栅格！
              open_.push(nc.sqdist, INTPOINT(nx, ny));
              nc.queueing = fwQueued;  //标记上已进入本次队列，避免重复进入
              nc.needsRaise = true;
              nc.obstX = invalidObstData;
              nc.obstY = invalidObstData;
              if (updateRealDist) {
                nc.dist = INFINITY;
              }
              nc.sqdist = INT_MAX;
              data_[nx][ny] = nc;
            } else {
              // HERE:筛选出无需更新的栅格点，让其加入到open_中，激发Low回波！或者Low波与Raise波的交会栅格边界处！
              // 第一：Raise波扩展中加入open_内的栅格（即if(){}内加入的），
              // 并不满足nc.queueing != fwQueued条件，因此不会重复加入
              // 第二：只有符合Raise波向外扩展方向，并且其最近障碍物有效的栅格，才会被加入，并在后续激发次生Low波
              // HERE:当然，假使原生Low波更快，并触及该邻接栅格，那么该邻接栅格已属于Low波内，不会重复进入队列！
              if (nc.queueing != fwQueued) {
                open_.push(nc.sqdist, INTPOINT(nx, ny));
                nc.queueing = fwQueued;
                data_[nx][ny] = nc;
              }
            }
          }
        }
      }
      c.needsRaise = false;
      // 8邻接栅格都处理完毕，形成wavefront
      c.queueing = bwProcessed;
      data_[x][y] = c;
    } else if (c.obstX != invalidObstData &&
               isOccupied(c.obstX, c.obstY, data_[c.obstX][c.obstY])) {
      // LOWER过程：从具有有效的最近障碍物的栅格开始，需要持续更新邻接栅格的最近障碍物信息
      c.queueing = fwProcessed;
      c.voronoi = occupied;

      for (int dx = -1; dx <= 1; dx++) {
        int nx = x + dx;
        if (nx <= 0 || nx >= sizeX_ - 1) continue;
        for (int dy = -1; dy <= 1; dy++) {
          if (dx == 0 && dy == 0) continue;
          int ny = y + dy;
          if (ny <= 0 || ny >= sizeY_ - 1) continue;
          dataCell nc = data_[nx][ny];
          if (!nc.needsRaise) {
            int distx = nx - c.obstX;
            int disty = ny - c.obstY;
            int newSqDistance = distx * distx + disty * disty;
            // 由于c栅格的最近障碍物有效，因此newSqDistance属于邻接栅格的sqdist备选
            bool overwrite = (newSqDistance < nc.sqdist);
            if (!overwrite && newSqDistance == nc.sqdist) {
              //如果nc没有最近障碍物，或者 nc的最近障碍物消失
              // HERE: 最近障碍物有两个备选，已选其中一项恰好被删除
              if (nc.obstX == invalidObstData ||
                  !isOccupied(nc.obstX, nc.obstY, data_[nc.obstX][nc.obstY])) {
                overwrite = true;
              }
            }
            if (overwrite) {
              open_.push(newSqDistance, INTPOINT(nx, ny));
              nc.queueing = fwQueued;
              if (updateRealDist) {
                nc.dist = sqrt((double)newSqDistance);
              }
              nc.sqdist = newSqDistance;
              nc.obstX = c.obstX;
              nc.obstY = c.obstY;
            } else {
              // HERE:
              // overwrite标志代表Low波是否抵达波形边界，是否继续向外扩展
              //所谓波形边界，表示多个Low波互相接触的栅格，适合判断voronoi栅格
              checkVoro(x, y, nx, ny, c, nc);
            }
            data_[nx][ny] = nc;
          }
        }
      }
    }
    data_[x][y] = c;
  }
}

float DynamicVoronoi::getDistance(int x, int y) {
  if ((x > 0) && (x < sizeX_) && (y > 0) && (y < sizeY_)) {
    return data_[x][y].dist;
  } else
    return -INFINITY;
}

bool DynamicVoronoi::isVoronoi(int x, int y) {
  dataCell c = data_[x][y];
  // HERE:只有如下状态被视为voronoi point，而且是pruned之后
  return (c.voronoi == free || c.voronoi == voronoiKeep);
}

bool DynamicVoronoi::isVoronoiAlternative(int x, int y) {
  int v = alternativeDiagram_[x][y];
  return (v == free || v == voronoiKeep);
}

//将发生状态变化（占用<-->不占用）的元素加入open_优先队列
void DynamicVoronoi::commitAndColorize(bool updateRealDist) {
  // addList_和removeList_中是触发Voronoi更新的元素，因此都要加入open_
  for (unsigned int i = 0; i < addList_.size(); i++) {
    INTPOINT p = addList_[i];
    int x = p.x;
    int y = p.y;
    dataCell c = data_[x][y];
    // HERE:保护措施，利用fwQueued状态，避免重复进入队列
    // TODO:这里感觉需要区分一下是否边界点，才能减少open_的大小
    if (c.queueing != fwQueued) {
      if (updateRealDist) {
        c.dist = 0;
      }
      c.sqdist = 0;
      c.obstX = x;
      c.obstY = y;
      c.queueing = fwQueued;  //已加入open_优先队列
      c.voronoi = occupied;
      data_[x][y] = c;
      //加入open_优先队列，加入open_的都是要更新的
      open_.push(0, INTPOINT(x, y));
    }
  }

  for (unsigned int i = 0; i < removeList_.size(); i++) {
    INTPOINT p = removeList_[i];
    int x = p.x;
    int y = p.y;
    dataCell c = data_[x][y];

    //在removeObstacle()处加入removeList_时已经解除占用状态；
    //如再次出现占用状态，则代表也在addList_内，无需重复处理
    if (isOccupied(x, y, c)) {
      continue;  // obstacle was removed and reinserted
    }
    open_.push(0, INTPOINT(x, y));  //加入open_优先队列
    if (updateRealDist) {
      c.dist = INFINITY;
    }
    c.sqdist = INT_MAX;
    //占用被清除，需要更新最近障碍物的距离-增加
    c.needsRaise = true;
    data_[x][y] = c;
  }
  removeList_.clear();
  addList_.clear();
}

// REVIEW:
/*
实现一算法，同时将原始驾驶路径的左右等高线栅格坐标给梳理出来，并且实现重采样和栅格引导排序等功能：
1. 在原始路径初始段，拿到左右两个最靠起始位置的栅格种子
2.
其中栅格的左右方向判断，主要依赖两个向量的叉乘：栅格与其最近障碍物构成的向量，附近原始轨迹的前行引导向量
3. 通过左右方向判断，将栅格方位分别纳入到两个容器内，以示区分
4. 再从种子栅格出发，遵循引导方向来采样下一个栅格方位，使其满足栅格间距的要求
5. 在栅格采样过程中，一旦引导向量的终点离栅格更近时，需要更新引导向量
6.
这样一直采样下去之后，将得到左右两个有序排列的栅格序列，每个栅格序列所显示的方位坐标，可以作为粗略的车道引导线
7.
上述粗略引导线，将利用离散点优化算法来使其满足车辆运动学的行驶条件，并交由轨迹规划模块来进行lattice采样
8.
前后引导向量的航向变化趋势代表了车道的弯道趋势，可以利用该值，来自适应调节左右栅格的避让空间
*/

// 利用栅格与其最近障碍物方位形成一向量，代表着Low波的法线方向，暂记为a；
// 将附近原始路径点与其超前点（距离和偏航角插值作为判断）形成代表原始驾驶路径的引导方向，暂记为b。
// 通过检测两者的叉乘符号，得到左右筛选；

void DynamicVoronoi::checkVoro(int x, int y, int nx, int ny, dataCell& c,
                               dataCell& nc) {
  // HERE:剔除掉障碍物占用，并且没有被Low波更新出有效最近障碍物的栅格
  if ((c.sqdist > 1 || nc.sqdist > 1) && nc.obstX != invalidObstData) {
    // HERE:c和nc的最近障碍物A和B，不是邻接关系才能真正判断（只有A和B分属c+nc栅格区域的不同侧，才能满足voronoi的定义！）
    if (abs(c.obstX - nc.obstX) > 1 || abs(c.obstY - nc.obstY) > 1) {
      // compute dist from x,y to obstacle of nx,ny
      int dxy_x = x - nc.obstX;
      int dxy_y = y - nc.obstY;
      int sqdxy = dxy_x * dxy_x + dxy_y * dxy_y;
      int stability_xy = sqdxy - c.sqdist;
      //必须在栅格的最近障碍物已经寻找到之后才能去寻找voronoi图
      if (sqdxy - c.sqdist < 0) return;

      // compute dist from nx,ny to obstacle of x,y
      int dnxy_x = nx - c.obstX;
      int dnxy_y = ny - c.obstY;
      int sqdnxy = dnxy_x * dnxy_x + dnxy_y * dnxy_y;
      int stability_nxy = sqdnxy - nc.sqdist;
      //同上，必须已经找到最近障碍物！
      if (sqdnxy - nc.sqdist < 0) return;

      // HERE:sqdist大于2，进一步筛选掉一些狭窄通道
      // which cell is added to the Voronoi diagram?
      if (stability_xy <= stability_nxy && c.sqdist > 2) {
        if (c.voronoi != free) {
          c.voronoi = free;
          reviveVoroNeighbors(x, y);
          pruneQueue_.push(INTPOINT(x, y));
        }
      }
      if (stability_nxy <= stability_xy && nc.sqdist > 2) {
        if (nc.voronoi != free) {
          nc.voronoi = free;
          reviveVoroNeighbors(nx, ny);
          pruneQueue_.push(INTPOINT(nx, ny));
        }
      }
    }
  }
}

void DynamicVoronoi::reviveVoroNeighbors(int& x, int& y) {
  for (int dx = -1; dx <= 1; dx++) {
    int nx = x + dx;
    if (nx <= 0 || nx >= sizeX_ - 1) continue;
    for (int dy = -1; dy <= 1; dy++) {
      if (dx == 0 && dy == 0) continue;
      int ny = y + dy;
      if (ny <= 0 || ny >= sizeY_ - 1) continue;
      dataCell nc = data_[nx][ny];
      // FIXME:此处利用voronoi标记来实现重复性剔除？
      if (nc.sqdist != INT_MAX && !nc.needsRaise &&
          (nc.voronoi == voronoiKeep || nc.voronoi == voronoiPrune)) {
        nc.voronoi = free;
        data_[nx][ny] = nc;
        pruneQueue_.push(INTPOINT(nx, ny));
      }
    }
  }
}

bool DynamicVoronoi::isOccupied(int x, int y) {
  dataCell c = data_[x][y];
  return (c.obstX == x && c.obstY == y);
}

bool DynamicVoronoi::isOccupied(int& x, int& y, dataCell& c) {
  //如果记录的障碍物坐标为自身，那么代表被占用
  return (c.obstX == x && c.obstY == y);
}

void DynamicVoronoi::visualize(const char* filename) {
  // write ppm files

  FILE* F = fopen(filename, "w");
  if (!F) {
    std::cerr << "could not open 'result.ppm' for writing!\n";
    return;
  }
  fprintf(F, "P6\n#\n");
  fprintf(F, "%d %d\n255\n", sizeX_, sizeY_);

  // fputc()执行3次，其实是依次对一个像素的RGB颜色赋值
  for (int y = sizeY_ - 1; y >= 0; y--) {
    for (int x = 0; x < sizeX_; x++) {
      unsigned char c = 0;
      if (alternativeDiagram_ != NULL &&
          (alternativeDiagram_[x][y] == free ||
           alternativeDiagram_[x][y] == voronoiKeep)) {
        //和alternative模式相关，先不用管
        fputc(255, F);
        fputc(0, F);
        fputc(0, F);
      } else if (isVoronoi(x, y)) {  //画Voronoi边
        fputc(0, F);
        fputc(0, F);
        fputc(255, F);
      } else if (data_[x][y].sqdist == 0) {  //填充障碍物
        fputc(0, F);
        fputc(0, F);
        fputc(0, F);
      } else {  //填充Voronoi区块内部
        float f = 80 + (sqrt(data_[x][y].sqdist) * 10);
        if (f > 255) f = 255;
        if (f < 0) f = 0;
        c = (unsigned char)f;
        fputc(c, F);
        fputc(c, F);
        fputc(c, F);
      }
    }
  }
  fclose(F);
}

void DynamicVoronoi::prune() {
  // filler
  //先遍历pruneQueue_中的元素，判断是否要加入到sortedPruneQueue_，
  //这一步的目的是合并紧邻的Voronoi边，将2条边夹着的栅格也设置为备选
  //再遍历sortedPruneQueue_中的元素，判断其是剪枝、保留、重试。
  while (!pruneQueue_.empty()) {
    INTPOINT p = pruneQueue_.front();
    pruneQueue_.pop();
    int x = p.x;
    int y = p.y;

    if (data_[x][y].voronoi == occupied)
      continue;  //如果(x,y)是occupied，无需处理，不可能是Voronoi
    if (data_[x][y].voronoi == freeQueued)
      continue;  //如果(x,y)是freeQueued，已经加入到sortedPruneQueue_，略过

    data_[x][y].voronoi = freeQueued;
    sortedPruneQueue_.push(data_[x][y].sqdist, p);

    /* tl t tr
       l c r
       bl b br */

    dataCell tr, tl, br, bl;
    tr = data_[x + 1][y + 1];
    tl = data_[x - 1][y + 1];
    br = data_[x + 1][y - 1];
    bl = data_[x - 1][y - 1];

    dataCell r, b, t, l;
    r = data_[x + 1][y];
    l = data_[x - 1][y];
    t = data_[x][y + 1];
    b = data_[x][y - 1];

    //文章只提了对待考察栅格判断是否符合模式，这里为什么要对待考察栅格的上下左右4个邻居栅格都判断呢？
    //我认为判断模式的目的就是将Voronoi边夹着的、包裹的栅格置为备选，因为待考察栅格是备选了，才使得周围栅格可能会被Voronoi边包裹，所以才要逐一检查。

    if (x + 2 < sizeX_ && r.voronoi == occupied) {
      // fill to the right
      //如果r的上下左右4个元素都!=occupied，对应文章的P38模式
      //    | ? | 1 | ? |
      //    | 1 |   | 1 |
      //    | ? | 1 | ? |
      if (tr.voronoi != occupied && br.voronoi != occupied &&
          data_[x + 2][y].voronoi != occupied) {
        r.voronoi = freeQueued;
        sortedPruneQueue_.push(r.sqdist, INTPOINT(x + 1, y));
        data_[x + 1][y] = r;
      }
    }
    if (x - 2 >= 0 && l.voronoi == occupied) {
      // fill to the left
      //如果l的上下左右4个元素都!=occupied
      if (tl.voronoi != occupied && bl.voronoi != occupied &&
          data_[x - 2][y].voronoi != occupied) {
        l.voronoi = freeQueued;
        sortedPruneQueue_.push(l.sqdist, INTPOINT(x - 1, y));
        data_[x - 1][y] = l;
      }
    }
    if (y + 2 < sizeY_ && t.voronoi == occupied) {
      // fill to the top
      //如果t的上下左右4个元素都!=occupied
      if (tr.voronoi != occupied && tl.voronoi != occupied &&
          data_[x][y + 2].voronoi != occupied) {
        t.voronoi = freeQueued;
        sortedPruneQueue_.push(t.sqdist, INTPOINT(x, y + 1));
        data_[x][y + 1] = t;
      }
    }
    if (y - 2 >= 0 && b.voronoi == occupied) {
      // fill to the bottom
      //如果b的上下左右4个元素都!=occupied
      if (br.voronoi != occupied && bl.voronoi != occupied &&
          data_[x][y - 2].voronoi != occupied) {
        b.voronoi = freeQueued;
        sortedPruneQueue_.push(b.sqdist, INTPOINT(x, y - 1));
        data_[x][y - 1] = b;
      }
    }
  }

  while (!sortedPruneQueue_.empty()) {
    INTPOINT p = sortedPruneQueue_.pop();
    dataCell c = data_[p.x][p.y];
    int v = c.voronoi;
    if (v != freeQueued && v != voronoiRetry) {  // || v>free || v==voronoiPrune
                                                 // || v==voronoiKeep) {
      //      assert(v!=retry);
      continue;
    }

    markerMatchResult r = markerMatch(p.x, p.y);
    if (r == pruned) {
      c.voronoi = voronoiPrune;  //对(x,y)即c剪枝
    } else if (r == keep) {
      c.voronoi = voronoiKeep;  //对(x,y)即c保留，成为Voronoi的边
    } else {
      c.voronoi = voronoiRetry;
      pruneQueue_.push(p);
    }
    data_[p.x][p.y] = c;

    //把需要retry的元素由pruneQueue_转移到sortedPruneQueue_
    //这样可以继续本while()循环，直到pruneQueue_和sortedPruneQueue_都为空
    if (sortedPruneQueue_.empty()) {
      while (!pruneQueue_.empty()) {
        INTPOINT p = pruneQueue_.front();
        pruneQueue_.pop();
        sortedPruneQueue_.push(data_[p.x][p.y].sqdist, p);
      }
    }
  }
}

void DynamicVoronoi::updateAlternativePrunedDiagram() {
  if (alternativeDiagram_ == NULL) {
    alternativeDiagram_ = new int*[sizeX_];
    for (int x = 0; x < sizeX_; x++) {
      alternativeDiagram_[x] = new int[sizeY_];
    }
  }

  std::queue<INTPOINT> end_cells;
  BucketPrioQueue<INTPOINT> sortedPruneQueue;
  for (int x = 1; x < sizeX_ - 1; x++) {
    for (int y = 1; y < sizeY_ - 1; y++) {
      dataCell& c = data_[x][y];
      alternativeDiagram_[x][y] = c.voronoi;
      if (c.voronoi <= free) {
        sortedPruneQueue.push(c.sqdist, INTPOINT(x, y));
        end_cells.push(INTPOINT(x, y));
      }
    }
  }

  for (int x = 1; x < sizeX_ - 1; x++) {
    for (int y = 1; y < sizeY_ - 1; y++) {
      if (getNumVoronoiNeighborsAlternative(x, y) >= 3) {
        alternativeDiagram_[x][y] = voronoiKeep;
        sortedPruneQueue.push(data_[x][y].sqdist, INTPOINT(x, y));
        end_cells.push(INTPOINT(x, y));
      }
    }
  }

  for (int x = 1; x < sizeX_ - 1; x++) {
    for (int y = 1; y < sizeY_ - 1; y++) {
      if (getNumVoronoiNeighborsAlternative(x, y) >= 3) {
        alternativeDiagram_[x][y] = voronoiKeep;
        sortedPruneQueue.push(data_[x][y].sqdist, INTPOINT(x, y));
        end_cells.push(INTPOINT(x, y));
      }
    }
  }

  while (!sortedPruneQueue.empty()) {
    INTPOINT p = sortedPruneQueue.pop();

    if (markerMatchAlternative(p.x, p.y)) {
      alternativeDiagram_[p.x][p.y] = voronoiPrune;
    } else {
      alternativeDiagram_[p.x][p.y] = voronoiKeep;
    }
  }

  // //delete worms
  while (!end_cells.empty()) {
    INTPOINT p = end_cells.front();
    end_cells.pop();

    if (isVoronoiAlternative(p.x, p.y) &&
        getNumVoronoiNeighborsAlternative(p.x, p.y) == 1) {
      alternativeDiagram_[p.x][p.y] = voronoiPrune;

      for (int dx = -1; dx <= 1; ++dx) {
        for (int dy = -1; dy <= 1; ++dy) {
          if (!(dx || dy) || (dx && dy)) {
            continue;
          }
          int nx = p.x + dx;
          int ny = p.y + dy;
          if (nx < 0 || nx >= sizeX_ || ny < 0 || ny >= sizeY_) {
            continue;
          }
          if (isVoronoiAlternative(nx, ny)) {
            if (getNumVoronoiNeighborsAlternative(nx, ny) == 1) {
              end_cells.push(INTPOINT(nx, ny));
            }
          }
        }
      }
    }
  }
}

bool DynamicVoronoi::markerMatchAlternative(int x, int y) {
  // prune if this returns true

  bool f[8];

  int nx, ny;
  int dx, dy;

  int i = 0;
  //  int obstacleCount=0;
  int voroCount = 0;
  for (dy = 1; dy >= -1; dy--) {
    ny = y + dy;
    for (dx = -1; dx <= 1; dx++) {
      if (dx || dy) {
        nx = x + dx;
        int v = alternativeDiagram_[nx][ny];
        bool b = (v <= free && v != voronoiPrune);
        //	if (v==occupied) obstacleCount++;
        f[i] = b;
        if (v <= free && !(dx && dy)) voroCount++;
        i++;
      }
    }
  }

  /*
   * 5 6 7
   * 3   4
   * 0 1 2
   */

  {
    // connected horizontal or vertically to only one cell
    if (voroCount == 1 && (f[1] || f[3] || f[4] || f[6])) {
      return false;
    }

    // 4-connected
    if ((!f[0] && f[1] && f[3]) || (!f[2] && f[1] && f[4]) ||
        (!f[5] && f[3] && f[6]) || (!f[7] && f[6] && f[4]))
      return false;

    if ((f[3] && f[4] && !f[1] && !f[6]) || (f[1] && f[6] && !f[3] && !f[4]))
      return false;
  }
  return true;
}

int DynamicVoronoi::getNumVoronoiNeighborsAlternative(int x, int y) {
  int count = 0;
  for (int dx = -1; dx <= 1; dx++) {
    for (int dy = -1; dy <= 1; dy++) {
      if ((dx == 0 && dy == 0) || (dx != 0 && dy != 0)) {
        continue;
      }

      int nx = x + dx;
      int ny = y + dy;
      if (nx < 0 || nx >= sizeX_ || ny < 0 || ny >= sizeY_) {
        continue;
      }
      if (alternativeDiagram_[nx][ny] == free ||
          alternativeDiagram_[nx][ny] == voronoiKeep) {
        count++;
      }
    }
  }
  return count;
}

//根据(x,y)邻接栅格的连接模式，判断是否要对(x,y)剪枝
DynamicVoronoi::markerMatchResult DynamicVoronoi::markerMatch(int x, int y) {
  // implementation of connectivity patterns
  bool f[8];
  int nx, ny;
  int dx, dy;
  int i = 0;
  // voroCount是对所有邻居栅格的统计，voroCountFour是对上下左右4个邻居栅格的统计
  int voroCount = 0;
  int voroCountFour = 0;

  for (dy = 1; dy >= -1; dy--) {
    ny = y + dy;
    for (dx = -1; dx <= 1; dx++) {
      if (dx || dy) {  //不考虑(x,y)点
        nx = x + dx;
        dataCell nc = data_[nx][ny];
        int v = nc.voronoi;
        //既不是occupied又不是voronoiPrune，即可能保留的栅格
        bool b = (v <= free && v != voronoiPrune);
        f[i] = b;
        if (b) {
          voroCount++;
          if (!(dx && dy)) {  //对上下左右4个点
            voroCountFour++;
          }
        }
        i++;
      }
    }
  }
  // i和位置的对应关系如下：
  //    | 0 | 1 | 2 |
  //    | 3 |   | 4 |
  //    | 5 | 6 | 7 |
  // 8个邻居栅格中最多有2个，上下左右只有1个可能保留的栅格
  if (voroCount < 3 && voroCountFour == 1 && (f[1] || f[3] || f[4] || f[6])) {
    return keep;
  }

  // 4-connected
  //    | 0 | 1 | ? |               | ? | 1 | 0 |             | ? | ? | ? | | ?
  //    | ? | ? | | 1 |   | ? |               | ? |   | 1 |             | 1 | |
  //    ? |             | ? |   | 1 | | ? | ? | ? |               | ? | ? | ? |
  //    | 0 | 1 | ? |             | ? | 1 | 0 |
  //对应《Efficient Grid-Based Spatial Representations for Robot Navigation in
  // Dynamic Environments》中的4-connected P14模式，旋转3次90度
  if ((!f[0] && f[1] && f[3]) || (!f[2] && f[1] && f[4]) ||
      (!f[5] && f[3] && f[6]) || (!f[7] && f[6] && f[4]))
    return keep;

  //    | ? | 0 | ? |                       | ? | 1 | ? |
  //    | 1 |   | 1 |                       | 0 |   | 0 |
  //    | ? | 0 | ? |                       | ? | 1 | ? |
  //对应文章中的4-connected P24模式，旋转1次90度
  if ((f[3] && f[4] && !f[1] && !f[6]) || (f[1] && f[6] && !f[3] && !f[4]))
    return keep;

  // keep voro cells inside of blocks and retry later
  //(x,y)周围可能保留的栅格很多，此时无法判断是否要对(x,y)剪枝
  if (voroCount >= 5 && voroCountFour >= 3 &&
      data_[x][y].voronoi != voronoiRetry) {
    return retry;
  }

  return pruned;
}
