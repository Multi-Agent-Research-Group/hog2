/*
 *  Created by Thayne Walker.
 *  Copyright 2017 All rights reserved.
 *
 * This file is part of HOG2.
 *
 * HOG2 is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 * 
 * HOG2 is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 * 
 * You should have received a copy of the GNU General Public License
 * along with HOG; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
 *
 */
#ifndef AnyAngleSipp_H
#define AnyAngleSipp_H

#include "PositionalUtils.h"
#include <iostream>
#include "FPUtil.h"

#include <vector>
#include <list>
#include <unordered_map>
#include <algorithm>

#define CN_EPSILON      1e-5
#define CN_INFINITY	1000000
#define CN_BT_G_MAX     1
#define CN_BT_G_MIN     2

typedef std::pair<double,double> Interval;
typedef std::vector<Interval> Intervals;

// Based on the algorithm of (Yakovlev and Andreychuk 2017)
template <typename Map, typename State>
class AnyAngleSipp
{
  struct constraint
  {
      constraint():x(0),y(0),g(0),agent(0),goal(0){}
      constraint(uint16_t x_, uint16_t y_, double t):x(x_),y(y_),g(t),agent(0),goal(0){}
      uint16_t x;
      uint16_t y;
      double g;
      int agent;
      bool goal;
  };

  struct ResultPathInfo
  {
    bool pathfound;
    float pathlength;
    unsigned int nodescreated;
    unsigned int numberofsteps;
    double time;
    std::list<State> path;
    std::vector<State> sections;

    ResultPathInfo()
    {
      nodescreated=0;
      numberofsteps=0;
      time=0;
      pathfound = false;
      pathlength = 0;
      path.clear();
      sections.clear();
    }
  };

  struct SearchResult
  {
    bool pathfound;
    double pathlength;
    unsigned int nodescreated;
    unsigned int numberofsteps;
    double time;
    unsigned int agents;
    int agentsSolved;
    std::vector<ResultPathInfo> pathInfo;

    SearchResult() : pathInfo(1)
    {
      pathfound = false;
      pathlength = 0;
      nodescreated = 0;
      numberofsteps = 0;
      time = 0;
      agents = 0;
    }

    ~SearchResult()
    {
      pathInfo.clear();
    }

  };

public:

    AnyAngleSipp(double weight, bool breakingties);
    ~AnyAngleSipp();
    void GetPath(std::vector<State>& solution, State const& s, State const& g, Map &map);
    void findSuccessors(State const& curState, State const& goal, Map const& map, std::list<State> &succs);
    Intervals const& getSafeIntervals(uint16_t x,uint16_t y) const;
    void setSafeIntervals(uint16_t x,uint16_t y,Intervals const& intvls);
    void setConstraint(uint16_t x, uint16_t y, double t);

private:

    void addOpen(State &newState);
    State findMin(int size);
    bool stopCriterion();
    double calculateDistanceFromCellToCell(double start_i, double start_j, double fin_i, double fin_j);
    bool lineOfSight(int i1, int j1, int i2, int j2, Map const& map);
    bool lineOfSight2(int i1, int j1, int i2, int j2, Map const& map);
    bool lineOfSight3(int i1, int j1, int i2, int j2, Map const& map);
    void makePrimaryPath(State curState);
    void makeSecondaryPath(State curState);
    void calculateLineSegment(std::vector<State> &line, State const& start, State const& goal);
    void addConstraints(int curAgent);
    void addConstraints(std::vector<State> const& path);
    State resetParent(State current, State Parent, Map const& map);
    bool findPath(std::vector<State>& solution, State const& s, State const& g, Map const& map);
    std::vector<std::pair<int, int> > findConflictCells(State cur);
    std::vector<std::pair<double, double> > findIntervals(State curState, std::vector<double> &EAT, int w);
    //std::vector<conflict> CheckConflicts();//bruteforce checker. It splits final(already built) trajectories into sequences of points and checks distances between them

    //std::vector<std::vector<std::vector<std::pair<double,double>>>> safe_intervals;
    std::unordered_map<uint32_t,Intervals> safe_intervals;
    uint32_t hash(uint16_t x,uint16_t y) const {return (x<<16)+y;}
    
    //std::vector<std::vector<std::vector<constraint>>> ctable;
    std::unordered_map<uint32_t,std::vector<constraint>> ctable;
    double weight;
    bool breakingties;
    unsigned int closeSize, openSize;
    std::list<State> *open, lppath;
    std::unordered_multimap<int, State> close;
    std::vector<State> hppath;
    double gap;
    int NUMOFCUR;
    SearchResult sresult;
    // Note: this should be static
    Intervals defaultIntervals;
};

inline bool sort_function(std::pair<double, double> a, std::pair<double, double> b)
{
    return fless(a.first, b.first);
}


template <typename Map, typename State>
AnyAngleSipp<Map,State>::AnyAngleSipp(double weight, bool breakingties)
{
    this->weight = weight;
    this->breakingties = breakingties;
    closeSize = 0;
    openSize = 0;
    gap = 2;//equivalent of 4r
    // This is used everywhere that a "save" interval is not defined
    defaultIntervals.push_back({0,CN_INFINITY});
}

template <typename Map, typename State>
AnyAngleSipp<Map,State>::~AnyAngleSipp()
{
}

template <typename Map, typename State>
bool AnyAngleSipp<Map,State>::stopCriterion()
{
    if(openSize == 0)
    {
        std::cout << "OPEN list is empty! " << std::endl;
        return true;
    }
    return false;
}

template <class Map, typename State>
void AnyAngleSipp<Map,State>::findSuccessors(State const& curState, State const& goal, Map const& map, std::list<State> &succs)
{
    State newState;
    std::vector<double> EAT;
    std::vector<std::pair<double, double>> intervals;
    double h_value;
    // Added this line
    //close.insert({curState.x * map.GetMapWidth() + curState.y, curState});
    auto parent = &(close.find(curState.x*map.GetMapWidth() + curState.y)->second);
    for(int i = -1; i <= +1; i++)
    {
        for(int j = -1; j <= +1; j++)
        {
            if((i+j) != 0 && i*j == 0 && map.IsTraversable(curState.x + i, curState.y + j))
            {
                newState.x = curState.x + i;
                newState.y = curState.y + j;
                newState.g = curState.g + 1;
                newState.Parent = parent;
                EAT.clear();
                h_value = weight*Util::distance(newState.x, newState.y, goal.x, goal.y);
                intervals = findIntervals(newState, EAT, map.GetMapWidth());
                for(int k = 0; k < intervals.size(); k++)
                {
                    newState.interval = intervals[k];
                    newState.g = EAT[k];
                    newState.F = newState.g + h_value;
                    succs.push_front(newState);
                }
                newState = resetParent(newState, curState, map);
                if(newState.Parent->x != parent->x || newState.Parent->y != parent->y)
                {
                    EAT.clear();
                    intervals = findIntervals(newState, EAT, map.GetMapWidth());
                    for(int k = 0; k < intervals.size(); k++)
                    {
                        newState.interval = intervals[k];
                        newState.g = EAT[k];
                        newState.F = newState.g + h_value;
                        succs.push_front(newState);
                    }
                }
            }
        }
    }
}

template <typename Map, typename State>
bool AnyAngleSipp<Map,State>::lineOfSight(int i1, int j1, int i2, int j2, Map const& map)
{
    int delta_i = std::abs(i1 - i2);
    int delta_j = std::abs(j1 - j2);
    int step_i = (i1 < i2 ? 1 : -1);
    int step_j = (j1 < j2 ? 1 : -1);
    int error = 0;
    int i = i1;
    int j = j1;
    int sep_value = delta_i*delta_i + delta_j*delta_j;
    if(delta_i == 0)
    {
        for(; j != j2; j += step_j)
            if(!map.IsTraversable(i, j))
                return false;
    }
    else if(delta_j == 0)
    {
        for(; i != i2; i += step_i)
            if(map.IsTraversable(i, j))
                return false;
    }
    else if(delta_i > delta_j)
    {
        for(; i != i2; i += step_i)
        {
            if(!map.IsTraversable(i, j))
                return false;
            if(!map.IsTraversable(i, j + step_j))
                return false;
            error += delta_j;
            if(error > delta_i)
            {
                if(((error << 1) - delta_i - delta_j)*((error << 1) - delta_i - delta_j) < sep_value)
                    if(!map.IsTraversable(i + step_i, j))
                        return false;
                if((3*delta_i - ((error << 1) - delta_j))*(3*delta_i - ((error << 1) - delta_j)) < sep_value)
                    if(!map.IsTraversable(i, j + 2*step_j))
                        return false;
                j += step_j;
                error -= delta_i;
            }
        }
        if(!map.IsTraversable(i, j))
            return false;
        if(!map.IsTraversable(i, j + step_j))
            return false;
    }
    else
    {
        for(; j != j2; j += step_j)
        {
            if(!map.IsTraversable(i, j))
                return false;
            if(!map.IsTraversable(i + step_i, j))
                return false;
            error += delta_i;
            if(error > delta_j)
            {
                if(((error << 1) - delta_i - delta_j)*((error << 1) - delta_i - delta_j) < sep_value)
                    if(!map.IsTraversable(i, j + step_j))
                        return false;
                if((3*delta_j - ((error << 1) - delta_i))*(3*delta_j - ((error << 1) - delta_i)) < sep_value)
                    if(!map.IsTraversable(i + 2*step_i, j))
                        return false;
                i += step_i;
                error -= delta_j;
            }
        }
        if(!map.IsTraversable(i, j))
            return false;
        if(!map.IsTraversable(i + step_i, j))
            return false;
    }
    return true;
}

template <typename Map, typename State>
double AnyAngleSipp<Map,State>::calculateDistanceFromCellToCell(double start_i, double start_j, double fin_i, double fin_j)
{
    return sqrt(double((start_i - fin_i)*(start_i - fin_i) + (start_j - fin_j)*(start_j - fin_j)));
}

template <typename Map, typename State>
State AnyAngleSipp<Map,State>::findMin(int size)
{
    State min;
    min.F = std::numeric_limits<float>::max();
    for(int i = 0; i < size; i++)
    {
        if(open[i].size() != 0 && open[i].begin()->F <= min.F)
        {
            if (open[i].begin()->F == min.F)
            {
                if (open[i].begin()->g >= min.g)
                    min=*open[i].begin();
            }
            else
                min=*open[i].begin();
        }
    }
    return min;
}

template <typename Map, typename State>
void AnyAngleSipp<Map,State>::addOpen(State &newState)
{
    typename std::list<State>::iterator iter;
    typename std::list<State>::iterator pos;
    bool posFound = false;
    pos = open[newState.x].end();
    if (open[newState.x].size() == 0)
    {
        open[newState.x].push_back(newState);
        openSize++;
        return;
    }
    for(iter = open[newState.x].begin(); iter != open[newState.x].end(); ++iter)
    {
        if ((iter->F >= newState.F) && (!posFound))
        {
            if (fabs(iter->F - newState.F) < CN_EPSILON)//CN_EPSILON is needed to prevent mistakes with comparison of double-type values
            {

                if ((newState.g > iter->g && breakingties == CN_BT_G_MAX) || (newState.g < iter->g && breakingties == CN_BT_G_MIN))
                {
                    pos = iter;
                    posFound = true;
                }
            }
            else
            {
                pos = iter;
                posFound = true;
            }
        }

        if (iter->x == newState.x && iter->y == newState.y && iter->interval.first == newState.interval.first)
        {
            if(iter->g - newState.g < CN_EPSILON)
                return;
            if(pos == iter)
            {
                iter->F = newState.F;
                iter->g = newState.g;
                iter->interval = newState.interval;
                iter->Parent = newState.Parent;
                return;
            }
            open[newState.x].erase(iter);
            openSize--;
            break;
        }
    }
    openSize++;
    open[newState.x].insert(pos, newState);
}

// Get safe intervals
template <typename Map, typename State>
Intervals const& AnyAngleSipp<Map,State>::getSafeIntervals(uint16_t x, uint16_t y) const{
  auto interval(safe_intervals.find(hash(x,y)));
  // If the interval is non-extant, return an interval of [0,INFTY).
  if(interval==safe_intervals.end()){
    return defaultIntervals;
  }else{
    return interval->second;
  }
}

// Add an unsafe interval to the safe intervals list
template <typename Map, typename State>
void AnyAngleSipp<Map,State>::setSafeIntervals(uint16_t x, uint16_t y, Intervals const& intvls){
  safe_intervals[hash(x,y)]=intvls;
}

// Add a constraint to the constraint table
template <typename Map, typename State>
void AnyAngleSipp<Map,State>::setConstraint(uint16_t x, uint16_t y, double time){
  constraint c(x,y,time);
  ctable[hash(x,y)].push_back(c);
}

// Initialize search
template <typename Map, typename State>
void AnyAngleSipp<Map,State>::GetPath(std::vector<State>& solution, State const& start, State const& goal, Map &map)
{
/*
#ifdef __linux__
    timeval begin, end;
    gettimeofday(&begin, NULL);
#else
    LARGE_INTEGER begin, end, freq;
    QueryPerformanceCounter(&begin);
    QueryPerformanceFrequency(&freq);
#endif
*/
    sresult.pathInfo.resize(1);
    sresult.agents = 1;
    sresult.agentsSolved = 0;
    //ctable.resize(map.GetMapHeight());
    // Note: it is not necessary to initialize safe intervals, we use a default case instead
    //safe_intervals.resize(map.GetMapHeight());
    //for(int i=0; i<map.GetMapHeight(); i++)
    //{
        //ctable[i].resize(map.GetMapWidth());
        //safe_intervals[i].resize(map.GetMapWidth());
        //for(int j=0; j<map.GetMapWidth(); j++)
        //{
            //ctable[i][j].resize(0);
            //safe_intervals[i][j].resize(0);
            //safe_intervals[i][j].push_back({0,CN_INFINITY});
        //}
    //}
    /*for(int i = 0; i < map.agents; i++)
    {
        map.addConstraint(map.start_i[i], map.start_j[i]);
        map.addConstraint(map.goal_i[i], map.goal_j[i]);
    }
    */
    if(findPath(solution, start, goal, map))
      addConstraints(0);
    close.clear();
    for(int i = 0; i< map.GetMapHeight(); i++)
      open[i].clear();
    delete [] open;
/*#ifdef __linux__
    gettimeofday(&end, NULL);
    sresult.time = (end.tv_sec - begin.tv_sec) + static_cast<double>(end.tv_usec - begin.tv_usec) / 1000000;
#else
    QueryPerformanceCounter(&end);
    sresult.time = static_cast<double long>(end.QuadPart-begin.QuadPart) / freq.QuadPart;
#endif
    std::vector<conflict> confs = CheckConflicts();
    for(int i = 0; i < confs.size(); i++)
        std::cout<<confs[i].x<<" "<<confs[i].y<<" "<<confs[i].g<<" "<<confs[i].agent1<<" "<<confs[i].agent2<<"\n";
    return sresult;*/
}


template <typename Map, typename State>
State AnyAngleSipp<Map,State>::resetParent(State current, State Parent, Map const& map)
{
    if(Parent.Parent == NULL || (current.x == Parent.Parent->x && current.y == Parent.Parent->y))
        return current;

    if(lineOfSight(Parent.Parent->x, Parent.Parent->y, current.x, current.y, map))
    {
        current.g = Parent.Parent->g + calculateDistanceFromCellToCell(Parent.Parent->x, Parent.Parent->y, current.x, current.y);
        current.Parent = Parent.Parent;
    }
    return current;
}

template <typename Map, typename State>
std::vector<std::pair<int,int>> AnyAngleSipp<Map,State>::findConflictCells(State cur)
{
    std::vector<std::pair<int,int>> cells(0);
    int i1 = cur.x, j1 = cur.y, i2 = cur.Parent->x, j2 = cur.Parent->y;
    int delta_i = std::abs(i1 - i2);
    int delta_j = std::abs(j1 - j2);
    int step_i = (i1 < i2 ? 1 : -1);
    int step_j = (j1 < j2 ? 1 : -1);
    int error = 0;
    int i = i1;
    int j = j1;
    int sep_value = delta_i*delta_i + delta_j*delta_j;
    if((delta_i + delta_j) == 0)//this situation is possible after modification of hppath and is needed for addConstraints function
        cells.push_back({i,j});
    else if(delta_i == 0)
        for(; j != j2+step_j; j += step_j)
            cells.push_back({i,j});
    else if(delta_j == 0)
        for(; i != i2+step_i; i += step_i)
            cells.push_back({i,j});
    else if(delta_i > delta_j)
    {
        for(; i != i2; i += step_i)
        {
            cells.push_back({i,j});
            cells.push_back({i,j+step_j});
            error += delta_j;
            if(error > delta_i)
            {
                j += step_j;
                if(((error << 1) - delta_i - delta_j)*((error << 1) - delta_i - delta_j) < sep_value)
                    cells.push_back({i + step_i,j - step_j});
                if((3*delta_i - ((error << 1) - delta_j))*(3*delta_i - ((error << 1) - delta_j)) < sep_value)
                    cells.push_back({i,j + step_j});
                error -= delta_i;
            }
        }
        cells.push_back({i,j});
        cells.push_back({i,j+step_j});
    }
    else
    {
        for(; j != j2; j += step_j)
        {
            cells.push_back({i,j});
            cells.push_back({i+step_i,j});
            error += delta_i;
            if(error > delta_j)
            {
                i += step_i;
                if(((error << 1) - delta_i - delta_j)*((error << 1) - delta_i - delta_j) < sep_value)
                    cells.push_back({i-step_i,j+step_j});
                if((3*delta_j - ((error << 1) - delta_i))*(3*delta_j - ((error << 1) - delta_i)) < sep_value)
                    cells.push_back({i+step_i,j});
                error -= delta_j;
            }
        }
        cells.push_back({i,j});
        cells.push_back({i+step_i,j});
    }
    return cells;
}


template <typename Map, typename State>
void AnyAngleSipp<Map,State>::addConstraints(int curAgent)
{
  addConstraints(sresult.pathInfo[curAgent].sections);
}

template <typename Map, typename State>
void AnyAngleSipp<Map,State>::addConstraints(std::vector<State> const& path)
{
    State cur;
    std::vector<std::pair<int,int>> cells;
    for(int a = 1; a < path.size(); a++)
    {
        cur = path[a];
        cells = findConflictCells(cur);
        int x1 = cur.x, y1 = cur.y, x0 = cur.Parent->x, y0 = cur.Parent->y;
        if(x1 != x0 || y1 != y0)
            cur.Parent->g = cur.g - calculateDistanceFromCellToCell(x0, y0, x1, y1);
        constraint add;
        add.agent = 0;
        int dx = abs(x1 - x0);
        int dy = abs(y1 - y0);
        if(dx == 0 && dy == 0)
        {
            add.x = cur.x;
            add.y = cur.y;
            uint32_t key(hash(add.x,add.y));
            add.goal = false;
            for(double i = cur.Parent->g; i <= cur.g; i += gap)
            {
                add.g = i;
                if(ctable[key].empty() || ctable[key].back().g != add.g)
                    ctable[key].push_back(add);
            }
            if(ctable[key].back().g < cur.g)
            {
                add.g = cur.g;
                ctable[key].push_back(add);
            }
            continue;
        }
        else
        {
            for(int i = 0; i < cells.size(); i++)
            {
                add.x = cells[i].first;
                add.y = cells[i].second;
                add.agent = 0;
                std::pair<double,double> ps,pg;
                ps={x0, y0};
                pg={x1, y1};
                double dist = fabs((ps.first - pg.first)*add.y + (pg.second - ps.second)*add.x + (ps.second*pg.first - ps.first*pg.second))
                        /sqrt(pow(ps.first - pg.first,2) + pow(ps.second - pg.second,2));
                double da = (x0 - add.x)*(x0 - add.x) + (y0 - add.y)*(y0 - add.y);
                double db = (x1 - add.x)*(x1 - add.x) + (y1 - add.y)*(y1 - add.y);
                double ha = sqrt(da - dist*dist);
                double hb = sqrt(db - dist*dist);
                constraint con;

                if(hb > 0)
                {
                    double lambda = ha/hb;
                    con.x = (x0 + lambda*x1)/(1 + lambda);
                    con.y = (y0 + lambda*y1)/(1 + lambda);
                }
                else
                {
                    con.x = x1;
                    con.y = y1;
                }
                con.g = cur.Parent->g + Util::distance(cur.Parent->x, cur.Parent->y, con.x, con.y);
                if(add.x == path.back().x && add.y == path.back().y)
                    con.goal = true;
                else
                    con.goal = false;
                con.agent = 0;
                uint32_t key(hash(add.x,add.y));
                if(ctable[key].empty() || fabs(ctable[key].back().g-con.g)>CN_EPSILON)
                    ctable[key].push_back(con);
            }
            for(int i = 0; i < cells.size(); i++)
            {
                add.x = cells[i].first;
                add.y = cells[i].second;
                add.agent = 0;
                Interval ps, pg, interval;
                ps = {x0, y0};
                pg = {x1, y1};
                double dist = fabs((ps.first - pg.first)*add.y + (pg.second - ps.second)*add.x + (ps.second*pg.first - ps.first*pg.second))
                        /sqrt(pow(ps.first - pg.first,2) + pow(ps.second - pg.second,2));
                double da = (x0 - add.x)*(x0 - add.x) + (y0 - add.y)*(y0 - add.y);
                double db = (x1 - add.x)*(x1 - add.x) + (y1 - add.y)*(y1 - add.y);
                double ha = sqrt(da - dist*dist);
                double hb = sqrt(db - dist*dist);
                double size = sqrt(1 - dist*dist);
                if(da >= 1 && db >= 1)
                {
                    interval.first = cur.Parent->g + ha - size;
                    interval.second = cur.Parent->g + ha + size;
                }
                else if(da < 1)
                {
                    interval.first = cur.Parent->g;
                    interval.second = cur.g - hb + size;
                }
                else
                {
                    interval.first = cur.Parent->g + ha - size;
                    interval.second = cur.g;
                }
                uint32_t key(hash(add.x,add.y));
                // If the safe interval does not explicitly exist, create it
                if(safe_intervals.find(key) == safe_intervals.end()){
                  safe_intervals[key] = defaultIntervals;
                }
                for(int j = 0; j < safe_intervals[key].size(); j++)
                {
                    if(safe_intervals[key][j].first <= interval.first && safe_intervals[key][j].second >= interval.first)
                    {
                        if(safe_intervals[key][j].second < interval.second)
                        {
                            safe_intervals[key].erase(safe_intervals[key].begin() + j);
                            break;
                        }
                        else if(safe_intervals[key][j].first == interval.first)
                        {
                            safe_intervals[key][j].first = interval.second;
                            break;
                        }
                        else if(safe_intervals[key][j].second <= interval.second)
                        {
                            safe_intervals[key][j].second = interval.first;
                            break;
                        }
                        else
                        {
                            std::pair<double,double> new1, new2;
                            new1.first = safe_intervals[key][j].first;
                            new1.second = interval.first;
                            new2.first = interval.second;
                            new2.second = safe_intervals[key][j].second;
                            safe_intervals[key].erase(safe_intervals[key].begin() + j);
                            safe_intervals[key].insert(safe_intervals[key].begin() + j, new2);
                            safe_intervals[key].insert(safe_intervals[key].begin() + j, new1);
                            break;
                        }
                    }
                    else if(safe_intervals[key][j].first > interval.first && safe_intervals[key][j].first < interval.second)
                    {
                        if(safe_intervals[key][j].second < interval.second)
                        {
                            safe_intervals[key].erase(safe_intervals[key].begin() + j);
                            break;
                        }
                        else
                        {
                            safe_intervals[key][j].first = interval.second;
                            break;
                        }
                    }
                }
            }
        }
    }
}

template <typename Map, typename State>
bool AnyAngleSipp<Map,State>::findPath(std::vector<State>& solution, State const& start, State const& goal, Map const& map)
{
/*
#ifdef __linux__
    timeval begin, end;
    gettimeofday(&begin, NULL);
#else
    LARGE_INTEGER begin, end, freq;
    QueryPerformanceCounter(&begin);
    QueryPerformanceFrequency(&freq);
#endif
*/
    open = new std::list<State>[map.GetMapHeight()];
    ResultPathInfo resultPath;
    openSize = 0;
    closeSize = 0;

    State curState(start);
    curState.F = weight * Util::distance(curState.x, curState.y, goal.x,goal.y);
    curState.interval = getSafeIntervals(curState.x,curState.y)[0];
    bool pathFound = false;
    open[curState.x].push_back(curState);
    openSize++;
    while(!stopCriterion())
    {
        curState = findMin(map.GetMapHeight());
        open[curState.x].pop_front();
        openSize--;
        close.insert({curState.x * map.GetMapWidth() + curState.y, curState});
        closeSize++;
        if(curState.x == goal.x && curState.y == goal.y && curState.interval.second == CN_INFINITY)
        {
            pathFound = true;
            break;
        }
        std::list<State> successors;
        successors.clear();
        findSuccessors(curState, goal, map, successors);
        for(auto it = successors.begin(); it != successors.end(); it++)
            addOpen(*it);
    }
    if(pathFound)
    {
        makePrimaryPath(curState);
        for(int i = 1; i < hppath.size(); i++)
            if((hppath[i].g - (hppath[i - 1].g + calculateDistanceFromCellToCell(hppath[i].x, hppath[i].y, hppath[i - 1].x, hppath[i - 1].y))) > 1e-4)
            {
                State add = hppath[i - 1];
                add.Parent = hppath[i].Parent;
                close.insert({add.x*map.GetMapWidth() + add.y, add});
                hppath[i].Parent = &(close.find(add.x*map.GetMapWidth() + add.y)->second);
                add.g = hppath[i].g - Util::distance(hppath[i].x, hppath[i].y, hppath[i - 1].x,hppath[i - 1].y);
                hppath.emplace(hppath.begin() + i, add);
                i++;
            }
        solution=hppath;
/*#ifdef __linux__
        gettimeofday(&end, NULL);
        resultPath.time = (end.tv_sec - begin.tv_sec) + static_cast<double>(end.tv_usec - begin.tv_usec) / 1000000;
#else
        QueryPerformanceCounter(&end);
        resultPath.time = static_cast<double long>(end.QuadPart-begin.QuadPart) / freq.QuadPart;
#endif*/
        resultPath.sections = hppath;
        makeSecondaryPath(curState);
        resultPath.nodescreated = openSize + closeSize;
        resultPath.pathfound = true;
        resultPath.path = lppath;
        resultPath.numberofsteps = closeSize;
        resultPath.pathlength = curState.g;
        sresult.pathfound = true;
        sresult.pathlength += curState.g;
        sresult.nodescreated += openSize + closeSize;
        sresult.numberofsteps += closeSize;
        //sresult.pathInfo[numOfCurAgent] = resultPath;
        sresult.agentsSolved++;
    }
    else
    {
/*#ifdef __linux__
        gettimeofday(&end, NULL);
        resultPath.time = (end.tv_sec - begin.tv_sec) + static_cast<double>(end.tv_usec - begin.tv_usec) / 1000000;
#else
        QueryPerformanceCounter(&end);
        resultPath.time = static_cast<double long>(end.QuadPart-begin.QuadPart) / freq.QuadPart;
#endif*/
        //std::cout<<numOfCurAgent<<" PATH NOT FOUND!\n";
        sresult.pathfound = false;
        sresult.nodescreated += closeSize;
        sresult.numberofsteps += closeSize;
        resultPath.nodescreated = closeSize;
        resultPath.pathfound = false;
        resultPath.path.clear();
        resultPath.sections.clear();
        resultPath.pathlength = 0;
        resultPath.numberofsteps = closeSize;
        //sresult.pathInfo[numOfCurAgent] = resultPath;
    }
    return resultPath.pathfound;
}

template <typename Map, typename State>
std::vector<std::pair<double,double>> AnyAngleSipp<Map,State>::findIntervals(State curState, std::vector<double> &EAT, int w)
{
    Intervals badIntervals(0), curStateIntervals(0);
    double ab = curState.g-curState.Parent->g;

    auto range = close.equal_range(curState.x * w + curState.y);
    bool has;
    Intervals const& curSafeIntervals(getSafeIntervals(curState.x,curState.y));
    for(int i = 0; i < curSafeIntervals.size(); i++)
        if(curSafeIntervals[i].second > curState.g && curSafeIntervals[i].first <= curState.Parent->interval.second + ab)
        {
            has = false;
            for(auto it = range.first; it != range.second; it++)
                if(it->second.interval.first == curSafeIntervals[i].first)
                {
                    has = true;
                    break;
                }
            if(!has)
            {
                curStateIntervals.push_back(curSafeIntervals[i]);
                if(curStateIntervals.back().first < curState.g)
                    EAT.push_back(curState.g);
                else
                    EAT.push_back(curStateIntervals.back().first);
            }
        }
    if(curStateIntervals.empty())
        return curStateIntervals;
    std::vector<std::pair<int,int>> cells = findConflictCells(curState);

    double da, db, offset, vi, vj, wi, wj, c1, c2, dist;
    Interval add;
    constraint con;
    for(int i = 0; i < cells.size(); i++)
    {
        uint32_t key(hash(cells[i].first,cells[i].second));
        for(int j = 0; j < ctable[key].size(); j++)
        {

            con = ctable[key][j];
            if(con.g + gap < curState.Parent->g && !con.goal)
                continue;
            da = (curState.x - con.x)*(curState.x - con.x) + (curState.y - con.y)*(curState.y - con.y);
            db = (curState.Parent->x - con.x)*(curState.Parent->x - con.x) + (curState.Parent->y - con.y)*(curState.Parent->y - con.y);
            vi = curState.Parent->x - curState.x;
            vj = curState.Parent->y - curState.y;
            wi = con.x - curState.x;
            wj = con.y - curState.y;
            c1 = vj*wj + vi*wi;
            c2 = vj*vj + vi*vi;
            if(c1 <= 0 || c2 <= c1)//if constraint is outside of the section
            {
                if(da < db)
                    dist = da;
                else
                    dist = db;
                if(dist < 1)//less than 2r
                {
                    if(dist == da)
                        add={con.g - gap, con.g + gap};
                    else
                        add={con.g - gap + ab, con.g + gap + ab};
                    if(con.goal == true)
                        add.second = CN_INFINITY;
                    badIntervals.push_back(add);
                    continue;
                }

            }
            else
            {
                dist = fabs(((curState.Parent->x - curState.x)*con.y + (curState.y - curState.Parent->y)*con.x
                             +(curState.Parent->y*curState.x - curState.Parent->x*curState.y)))/ab;
                if(dist < 1)
                {
                    offset = sqrt(da - dist*dist);
                    add = {con.g - gap + offset, con.g + gap + offset};
                    if(con.goal == true)
                        add.second = CN_INFINITY;
                    badIntervals.push_back(add);
                }
            }
        }
    }

    //combining and sorting bad intervals
    if(badIntervals.size() > 1)
    {
        std::sort(badIntervals.begin(), badIntervals.end(), sort_function);
        std::pair<double,double> cur, next;
        for(int i = 0; i < badIntervals.size() - 1; i++)
        {
            cur = badIntervals[i];
            next = badIntervals[i + 1];
            if((cur.first - next.first)*(cur.second - next.first) < CN_EPSILON)
            {
                if(next.second > cur.second)
                    badIntervals[i].second = next.second;
                badIntervals.erase(badIntervals.begin() + i + 1);
                i--;
            }
        }
    }

    //searching reachebale intervals and theirs EAT
    if(badIntervals.size() > 0)
    {
        for(int i = 0; i < badIntervals.size(); i++)
            for(int j = 0; j < curStateIntervals.size(); j++)
                if(badIntervals[i].first <= EAT[j])
                {
                    if(badIntervals[i].second >= curStateIntervals[j].second)
                    {
                        curStateIntervals.erase(curStateIntervals.begin() + j);
                        EAT.erase(EAT.begin() + j);
                        j--;
                        continue;
                    }
                    else if(badIntervals[i].second > EAT[j])
                        EAT[j] = badIntervals[i].second;
                }
        for(int i = 0; i < curStateIntervals.size(); i++)
            if(EAT[i] > curState.Parent->interval.second + ab || curStateIntervals[i].second < curState.g)
            {
                curStateIntervals.erase(curStateIntervals.begin() + i);
                EAT.erase(EAT.begin() + i);
                i--;
            }
    }
    return curStateIntervals;
}

/*template <typename Map, typename State>
std::vector<conflict> AnyAngleSipp<Map,State>::CheckConflicts()
{
    std::vector<conflict> conflicts(0);
    conflict conf;
    State cur, check;
    std::vector<std::vector<conflict>> positions;
    positions.resize(sresult.agents);
    for(int i = 0; i < sresult.agents; i++)
    {
        if(!sresult.pathInfo[i].pathfound)
            continue;
        positions[i].resize(0);
        int k = 0;
        double part = 1;
        for(int j = 1; j<sresult.pathInfo[i].sections.size(); j++)
        {
            cur = sresult.pathInfo[i].sections[j];
            check = sresult.pathInfo[i].sections[j-1];
            int di = cur.x - check.x;
            int dj = cur.y - check.y;
            double dist = (cur.g - check.g)*10;
            int steps = (cur.g - check.g)*10;
            if(dist - steps + part >= 1)
            {
                steps++;
                part = dist - steps;
            }
            else
                part += dist - steps;
            double stepi = double(di)/dist;
            double stepj = double(dj)/dist;
            double curg = double(k)*0.1;
            double curi = check.x + (curg - check.g)*di/(cur.g - check.g);
            double curj = check.y + (curg - check.g)*dj/(cur.g - check.g);
            conf.x = curi;
            conf.y = curj;
            conf.g = curg;
            positions[i].push_back(conf);
            k++;
            while(curg <= cur.g)
            {
                if(curg + 0.1 > cur.g)
                    break;
                curi += stepi;
                curj += stepj;
                curg += 0.1;
                conf.x = curi;
                conf.y = curj;
                conf.g = curg;
                positions[i].push_back(conf);
                k++;
            }
        }
        if(double(k - 1)*0.1 < sresult.pathInfo[i].sections.back().g)
        {
            conf.x = sresult.pathInfo[i].sections.back().x;
            conf.y = sresult.pathInfo[i].sections.back().y;
            conf.g = sresult.pathInfo[i].sections.back().g;
            positions[i].push_back(conf);
        }
    }
    int max = 0;
    for(int i = 0; i < positions.size(); i++)
        if(positions[i].size() > max)
            max = positions[i].size();
    for(int i = 0; i < sresult.agents; i++)
    {
        for(int k = 0; k < max; k++)
        {
            for(int j = i + 1; j < sresult.agents; j++)
            {
                if(!sresult.pathInfo[j].pathfound || !sresult.pathInfo[i].pathfound)
                    continue;
                conflict a, b;
                if(positions[i].size() > k)
                    a = positions[i][k];
                else
                    a = positions[i].back();
                if(positions[j].size() > k)
                    b = positions[j][k];
                else
                    b = positions[j].back();
                if(sqrt((a.x - b.x)*(a.x - b.x) + (a.y - b.y)*(a.y - b.y)) + CN_EPSILON < 1.0)
                {
                    std::cout<<a.x<<" "<<a.y<<" "<<b.x<<" "<<b.y<<" "<<sqrt((a.x - b.x)*(a.x - b.x) + (a.y - b.y)*(a.y - b.y))<<"\n";
                    conf.x = b.x;
                    conf.y = b.y;
                    conf.agent1 = i;
                    conf.agent2 = j;
                    conf.g = b.g;
                    conflicts.push_back(conf);
                }
            }
        }
    }
    return conflicts;
}*/

template <typename Map, typename State>
void AnyAngleSipp<Map,State>::makePrimaryPath(State curState)
{
    hppath.clear();
    hppath.shrink_to_fit();
    std::list<State> path;
    path.push_front(curState);
    if(curState.Parent != NULL)
    {
        curState = *curState.Parent;
        if(curState.Parent != NULL)
        {
            do
            {
                path.push_front(curState);
                curState = *curState.Parent;
            }
            while(curState.Parent != NULL);
        }
        path.push_front(curState);
    }
    for(auto it = path.begin(); it != path.end(); it++)
        hppath.push_back(*it);
    return;
}

template <typename Map, typename State>
void AnyAngleSipp<Map,State>::makeSecondaryPath(State curState)
{
    if(curState.Parent != NULL)
    {
        std::vector<State> lineSegment;
        do
        {
            calculateLineSegment(lineSegment, *curState.Parent, curState);
            lppath.insert(lppath.begin(), ++lineSegment.begin(), lineSegment.end());
            curState = *curState.Parent;
        }
        while(curState.Parent != NULL);
        lppath.push_front(*lineSegment.begin());
    }
    else
        lppath.push_front(curState);
}

template <typename Map, typename State>
void AnyAngleSipp<Map,State>::calculateLineSegment(std::vector<State> &line, State const& start, State const& goal)
{
    int i1 = start.x;
    int i2 = goal.x;
    int j1 = start.y;
    int j2 = goal.y;

    int delta_i = std::abs(i1 - i2);
    int delta_j = std::abs(j1 - j2);
    int step_i = (i1 < i2 ? 1 : -1);
    int step_j = (j1 < j2 ? 1 : -1);
    int error = 0;
    int i = i1;
    int j = j1;
    if (delta_i > delta_j)
    {
        for (; i != i2; i += step_i)
        {
            line.push_back(State(i,j));
            error += delta_j;
            if ((error << 1) > delta_i)
            {
                j += step_j;
                error -= delta_i;
            }
        }
    }
    else
    {
        for (; j != j2; j += step_j)
        {
            line.push_back(State(i,j));
            error += delta_i;
            if ((error << 1) > delta_j)
            {
                i += step_i;
                error -= delta_j;
            }
        }
    }
    return;
}
#endif // AnyAngleSipp_H
