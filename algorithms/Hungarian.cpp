/********************************************************************
 ********************************************************************
 ** C++ and STL class implementation of the Hungarian algorithm by David Schwarz, 2012
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

#include <algorithm>
#include <limits>
#include <assert.h>
#include "Hungarian.h"
#include <iostream>

#define verbose (0)

Hungarian::Hungarian():m_cost(0),m_rows(1),m_cols(1),m_costmatrix(1),m_assignment(1){}

// Input is input_matrix in rows, cols
Hungarian::Hungarian(const vector<vector<int> >& input_matrix, unsigned maxjobsperworker, MODE mode):m_cost(0),m_jobs(input_matrix.back().size()),m_maxjobsperworker(maxjobsperworker),m_rows(input_matrix.size()),m_cols(std::max((size_t)m_rows, input_matrix.back().size())),m_costmatrix(m_rows,vector<int>(m_cols)),m_assignment(m_rows,vector<int>(m_cols)) {
  // If rows was > cols, zeros should have been padded to make it square
  // If rows is < cols, that means there are more jobs than workers so we have to make
  //   multiple assignments. Thus we duplicate the assignees here and re-pad columns
  //   if necessary.
  if(m_rows<m_cols){
    unsigned newrows=m_cols+m_cols%m_rows;
    m_costmatrix.reserve(newrows);
    // Duplicate all rows
    for(unsigned r(0); r<newrows-m_rows; ++r){
      if(m_cols<newrows){
        m_costmatrix[r].resize(newrows);
      }
      m_costmatrix.push_back(m_costmatrix[r]);
    }
    m_cols=m_rows=newrows;
    m_assignment.resize(m_rows,vector<int>(m_cols));
  }

  unsigned i,j;
  int max_cost(0);

  for(i=0; i<m_rows; i++) 
  {
    for(j=0; j<m_cols; j++) 
    {
      m_costmatrix[i][j] =  (j < input_matrix.back().size()) ? input_matrix[i%input_matrix.size()][j] : 0;
      m_assignment[i][j] = 0;

      if(max_cost < m_costmatrix[i][j])
      {
        max_cost = m_costmatrix[i][j];
      }
    }
  }

  if(mode == HUNGARIAN_MODE_MAXIMIZE_UTIL) 
  {
    for(i=0; i<m_rows; i++) 
    {
      for(j=0; j<m_cols; j++) 
      {
        m_costmatrix[i][j] =  max_cost - m_costmatrix[i][j];
      }
    }
  }
}


void Hungarian::print_assignment() {
  hungarian_print_matrix(m_assignment);
}

void Hungarian::print_cost() {
  hungarian_print_matrix(m_costmatrix);
}

void Hungarian::print_status() 
{
  
  fprintf(stderr,"cost:\n");
  print_cost();

  fprintf(stderr,"assignment:\n");
  print_assignment();
  
}

bool Hungarian::check_solution(const vector<int>& row_dec, const vector<int>& col_inc, const vector<int>& col_vertex)
{
  int k, l, m, n;

  m = m_rows;
  n = m_cols;
  // Begin doublecheck the solution 23
  for (k=0;k<m;k++)
    for (l=0;l<n;l++)
      if(m_costmatrix[k][l]<row_dec[k]-col_inc[l])
        return false;

  for (k=0;k<m;k++)
  {
    l=col_vertex[k];
    if(l<0 || m_costmatrix[k][l]!=row_dec[k]-col_inc[l])
      return false;
  }
  k=0;

  for (l=0;l<n;l++)
  {
    if(col_inc[l])
    {
      k++;
    }
  }
  if(k>m)
  {
    return false;
  }

  //everything checks out, then
  return true;
  // End doublecheck the solution 23
}

void Hungarian::assign_solution(const vector<int>& row_dec,const vector<int>&  col_inc, const vector<int>&  col_vertex, std::vector<std::vector<unsigned>>& assignment,unsigned workers, bool multiplejobs){
  // End Hungarian algorithm 18
  int i, k, l;

  for (i=0;i<m_rows;++i)
  {
    m_assignment[i][col_vertex[i]]=HUNGARIAN_ASSIGNED;
    /*TRACE("%d - %d\n", i, col_vertex[i]);*/
  }
  for (k=0;k<m_rows;++k)
  {
    for (l=0;l<m_cols;++l)
    {
      /*TRACE("%d ",m_costmatrix[k][l]-row_dec[k]+col_inc[l]);*/
      m_costmatrix[k][l]=m_costmatrix[k][l]-row_dec[k]+col_inc[l];
    }
    /*TRACE("\n");*/
  }
  for (i=0;i<m_rows;i++)
  {
    m_cost+=row_dec[i];
  }
  for (i=0;i<m_cols;i++)
  {
    m_cost-=col_inc[i];
  }
  if(verbose)
    fprintf(stderr, "Cost is %d\n",m_cost);

  interpret(assignment, workers, multiplejobs);
  for( auto const& a:assignment){
    for( auto const& j:a){
      std::cout << j << " ";
    }
    std::cout << "\n";
  }

}

bool Hungarian::solve(std::vector<std::vector<unsigned>>& assignments, unsigned workers, bool multiplejobs){
  int i, j, m, n, k, l, s, t, q, unmatched, cost;

  m = m_rows;
  n = m_cols;

  static int INF = std::numeric_limits<int>::max();

  //vertex alternating paths,
  vector<int> col_vertex(m), row_vertex(n), unchosen_row(m), parent_row(n),
    row_dec(m),  col_inc(n),  slack_row(m),    slack(n);

  cost=0;

  for (i=0;i<m_rows;i++) 
  {
    col_vertex[i]=0;
    unchosen_row[i]=0;
    row_dec[i]=0;
    slack_row[i]=0;
  }

  for (j=0;j<m_cols;j++) 
  {
    row_vertex[j]=0;
    parent_row[j] = 0;
    col_inc[j]=0;
    slack[j]=0;
  }

  //Double check assignment matrix is 0
  m_assignment.assign(m, vector<int>(n, HUNGARIAN_NOT_ASSIGNED));

  // Begin subtract column minima in order to start with lots of zeroes 12
  if(verbose)
  {
    fprintf(stderr, "Using heuristic\n");
  }

  for (l=0;l<n;l++)
  {
    s = m_costmatrix[0][l];

    for (k=1;k<m;k++) 
    {
      if(m_costmatrix[k][l] < s)
      {
        s=m_costmatrix[k][l];
      }
      cost += s;
    }

    if(s!=0)
    {
      for (k=0;k<m;k++)
      {
        m_costmatrix[k][l]-=s;
      }
    }

    //pre-initialize state 16
    row_vertex[l]= -1;
    parent_row[l]= -1;
    col_inc[l]=0;
    slack[l]=INF;
  }
  // End subtract column minima in order to start with lots of zeroes 12

  // Begin initial state 16
  t=0;

  for (k=0;k<m;k++)
  {
    bool row_done = false;
    s=m_costmatrix[k][0];

    for (l=0;l<n;l++)
    {

      if(l > 0)
      {
        if(m_costmatrix[k][l] < s)
        {
          s = m_costmatrix[k][l];
        }
        row_dec[k]=s;
      }

      if(s == m_costmatrix[k][l] && row_vertex[l]<0)
      {
        col_vertex[k]=l;
        row_vertex[l]=k;

        if(verbose)
        {
          fprintf(stderr, "matching col %d==row %d\n",l,k);
        }
        row_done = true;
        break;
      }
    }

    if(!row_done)
    {
      col_vertex[k]= -1;

      if(verbose)
      {
        fprintf(stderr, "node %d: unmatched row %d\n",t,k);
      }

      unchosen_row[t++]=k;
    }

  }
  // End initial state 16

  bool checked = false;

  // Begin Hungarian algorithm 18

  //is matching already complete?
  if(t == 0)
  {
    checked = check_solution(row_dec, col_inc, col_vertex);
    if(checked)
    {
      //finish assignment, wrap up and done.
      assign_solution(row_dec, col_inc, col_vertex, assignments, workers, multiplejobs);
      return true;
    }
    else
    {
      if(verbose)
      {
        fprintf(stderr, "Could not solve. Error.\n");
      }
      return false;
    }
  }

  unmatched=t;


  while (1)
  {
    if(verbose)
    {
      fprintf(stderr, "Matched %d rows.\n",m-t);
    }
    q=0;
    bool try_matching;
    while (1)
    {
      while (q<t)
      {
        // Begin explore node q of the forest 19

        k=unchosen_row[q];
        s=row_dec[k];
        for (l=0;l<n;l++)
        {
          if(slack[l])
          {
            int del;
            del=m_costmatrix[k][l]-s+col_inc[l];
            if(del<slack[l])
            {
              if(del==0)
              {
                if(row_vertex[l]<0)
                {
                  goto breakthru;
                }
                slack[l]=0;
                parent_row[l]=k;
                if(verbose)
                  fprintf(stderr, "node %d: row %d==col %d--row %d\n",
                      t,row_vertex[l],l,k);
                unchosen_row[t++]=row_vertex[l];
              }
              else
              {
                slack[l]=del;
                slack_row[l]=k;
              }
            }
          }
        }
        // End explore node q of the forest 19
        q++;	
      }

      // Begin introduce a new zero into the matrix 21
      s=INF;
      for (l=0;l<n;l++)
      {
        if(slack[l] && slack[l]<s)
        {
          s=slack[l];
        }
      }
      for (q=0;q<t;q++)
      {
        row_dec[unchosen_row[q]]+=s;
      }
      for (l=0;l<n;l++)
      {
        //check slack
        if(slack[l])
        {
          slack[l]-=s;
          if(slack[l]==0)
          {
            // Begin look at a new zero 22
            k=slack_row[l];
            if(verbose)
            {
              fprintf(stderr, 
                  "Decreasing uncovered elements by %d produces zero at [%d,%d]\n",
                  s,k,l);
            }
            if(row_vertex[l]<0)
            {
              for (j=l+1;j<n;j++)
                if(slack[j]==0)
                {
                  col_inc[j]+=s;
                }

              goto breakthru;
            }
            else
            {
              parent_row[l]=k;
              if(verbose)
              {
                fprintf(stderr, "node %d: row %d==col %d--row %d\n",t,row_vertex[l],l,k);
                unchosen_row[t++]=row_vertex[l];
              }
            }
            // End look at a new zero 22
          }
        }
        else
        {
          col_inc[l]+=s;
        }
      }
      // End introduce a new zero into the matrix 21
    }

breakthru:
    // Begin update the matching 20
    if(verbose)
    {
      fprintf(stderr, "Breakthrough at node %d of %d!\n",q,t);
    }
    while (1)
    {
      j=col_vertex[k];
      col_vertex[k]=l;
      row_vertex[l]=k;
      if(verbose)
      {
        fprintf(stderr, "rematching col %d==row %d\n",l,k);
      }
      if(j<0)
      {
        break;
      }
      k=parent_row[j];
      l=j;
    }
    // End update the matching 20
    if(--unmatched == 0)
    {
      checked = check_solution(row_dec, col_inc, col_vertex);
      if(checked)
      {
        //finish assignment, wrap up and done.
        assign_solution(row_dec, col_inc, col_vertex, assignments, workers, multiplejobs);
        return true;
      }
      else
      {
        if(verbose)
        {
          fprintf(stderr, "Could not solve. Error.\n");
        }
        return false;
      }
    }

    // Begin get ready for another stage 17
    t=0;
    for (l=0;l<n;l++)
    {
      parent_row[l]= -1;
      slack[l]=INF;
    }
    for (k=0;k<m;k++)
    {
      if(col_vertex[k]<0)
      {
        if(verbose)
          fprintf(stderr, "node %d: unmatched row %d\n",t,k);
        unchosen_row[t++]=k;
      }
    }
    // End get ready for another stage 17
  }// back to while loop

}

//ACCESSORS

int Hungarian::cost() const
{
	return m_cost;
}

const vector<vector<int> >& Hungarian::assignment() const
{
	return m_assignment;
}

