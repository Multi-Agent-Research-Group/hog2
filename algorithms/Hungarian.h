/********************************************************************
 ********************************************************************
 ** C++ class implementation of the Hungarian algorithm by David Schwarz, 2012
 **
 **
 ** O(n^3) implementation derived from libhungarian by Cyrill Stachniss, 2004
 **
 **
 ** Solving the Minimum Assignment Problem using the 
 ** Hungarian Method.
 **
 ** ** This file may be freely copied and distributed! **
 **
 **
 ** This file is distributed in the hope that it will be useful,
 ** but WITHOUT ANY WARRANTY; without even the implied 
 ** warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
 ** PURPOSE.  
 **
 ********************************************************************
 ********************************************************************/

#include <iostream>
#include <vector>
#include <stdio.h>

#ifndef HUNGARIAN_H
#define HUNGARIAN_H

using std::vector;

typedef enum {
	HUNGARIAN_MODE_MINIMIZE_COST,
	HUNGARIAN_MODE_MAXIMIZE_UTIL,
} MODE;

typedef enum {
	HUNGARIAN_NOT_ASSIGNED,
	HUNGARIAN_ASSIGNED,
} ASSIGN;



class Hungarian
{

  public:
    /** This method initialize the hungarian_problem structure and init 
     *  the  cost matrices (missing lines or columns are filled with 0).
     *  It returns the size of the quadratic(!) assignment matrix. **/

    Hungarian();
    Hungarian(const vector<vector<int> >&, unsigned maxjobsperworker=1, MODE=MODE::HUNGARIAN_MODE_MINIMIZE_COST);

    /** This method computes the optimal assignment. **/
    bool solve(std::vector<std::vector<unsigned>>&,unsigned,bool multiplejobs=true);

    /** Accessor for the cost **/
    int cost() const;

    /** Reference accessor for assignment **/
    const vector<vector<int> >& assignment() const;

    /** Print the computed optimal assignment. **/
    void print_assignment();

    /** Print the cost matrix. **/
    void print_cost();

    /** Print cost matrix and assignment matrix. **/
    void print_status();


    inline void interpret(std::vector<std::vector<unsigned>>& assignments,unsigned workers, bool multiplejobs){
      static int INF = std::numeric_limits<int>::max();
      if(m_assignment.size()%workers){
        assert(!"Expected workers to be a multiple of assignments");
      }
      assignments.resize(workers);
      int worker(0);
      std::vector<int> best(workers,INF);
      for(auto const& a:m_assignment){
        auto pos(std::find(a.begin(),a.end(),1));
        if(pos!=a.end()){
          unsigned jobNum(pos-a.begin());
          if(m_jobs>jobNum){
            if(multiplejobs || m_costmatrix[worker][jobNum]<best[worker]){
              best[worker]=m_costmatrix[worker][jobNum];
              assignments[worker%workers].push_back(jobNum);
            }
          }
          ++worker;
        }
      }
    }

    static inline void hungarian_print_matrix(const vector<vector<int> >& C)
    {
      int i,j;
      fprintf(stderr , "\n");
      for(i=0; i<C.size(); i++) 
      {
        fprintf(stderr, " [");
        for(j=0; j<C.back().size(); j++) 
        {
          fprintf(stderr, "%5d ",C[i][j]);
        }
        fprintf(stderr, "]\n");
      }
      fprintf(stderr, "\n");
    }


  protected:
    bool check_solution(const vector<int>& row_dec, const vector<int>& col_inc, const vector<int>& col_vertex);
    void assign_solution(const vector<int>& row_dec, const vector<int>& col_inc, const vector<int>& col_vertex,std::vector<std::vector<unsigned>>&,unsigned,bool multiplejobs);

  private:

    int m_cost;
    unsigned m_jobs;
    unsigned m_maxjobsperworker;
    unsigned m_rows;
    unsigned m_cols;
    vector<vector<int> > m_costmatrix;
    vector<vector<int> > m_assignment;   

};

#endif



