#ifndef VERTEXCOVER_H
#define VERTEXCOVER_H
#include <iostream>
#include <vector>
int cover_size(std::vector<unsigned> const& cover)
{
  int count=0;
  for(int i=0; i<cover.size(); i++) if(cover[i]) count++;
  return count;
}

bool removable(std::vector<unsigned> const& neighbor, std::vector<unsigned>  const&cover)
{
  bool check=true;
  for(int i=0; i<neighbor.size(); i++)
    if(cover[neighbor[i]]==0)
    {
      check=false;
      break;
    }
  return check;
}

int max_removable(std::vector<std::vector<unsigned> > const& neighbors, std::vector<unsigned> const& cover)
{
  int r=-1, max=-1;
  for(int i=0; i<cover.size(); i++)
  {
    if(cover[i]==1 && removable(neighbors[i],cover)==true)
    {
      std::vector<unsigned> temp_cover=cover;
      temp_cover[i]=0;
      int sum=0;
      for(int j=0; j<temp_cover.size(); j++)
        if(temp_cover[j]==1 && removable(neighbors[j], temp_cover)==true)
          sum++;
      if(sum>max)
      {
        if(r==-1)
        {
          max=sum;
          r=i;
        }
        else if(neighbors[r].size()>=neighbors[i].size())
        {
          max=sum;
          r=i;
        }
      }
    }
  }
  return r;
}

std::vector<unsigned> procedure_1(std::vector<std::vector<unsigned>> const& neighbors, std::vector<unsigned> const& cover)
{
  std::vector<unsigned> temp_cover=cover;
  int r=0;
  while(r!=-1)
  {
    r= max_removable(neighbors,temp_cover);
    if(r!=-1) temp_cover[r]=0;
  }
  return temp_cover;
}

std::vector<unsigned> procedure_2(std::vector<std::vector<unsigned> > neighbors, std::vector<unsigned> cover)
{
  int count=0;
  std::vector<unsigned> temp_cover=cover;
  int i=0;
  for(int i=0; i<temp_cover.size(); i++)
  {
    if(temp_cover[i]==1)
    {
      int sum=0, index;
      for(int j=0; j<neighbors[i].size(); j++)
        if(temp_cover[neighbors[i][j]]==0) {index=j; sum++;}
      if(sum==1 && cover[neighbors[i][index]]==0)
      {
        temp_cover[neighbors[i][index]]=1;
        temp_cover[i]=0;
        temp_cover=procedure_1(neighbors,temp_cover);
        count++;
      }
    }
  }
  return temp_cover;
}


// get min size, max weight v-cover
void maxWeightVertexCover(std::vector<std::vector<unsigned>> const& graph, std::vector<std::vector<unsigned>>& covers){
  std::set<std::vector<unsigned>> res;
  auto n(graph.size()); 
  int i, j, p, q, r, s, min, counter=0;
  std::vector<std::vector<unsigned> > neighbors(n);
  for(i=0; i<graph.size(); i++)
  {
    for(j=0; j<graph[i].size(); j++){
      if(graph[i][j]) neighbors[i].push_back(j);
    }
  }
  min=n+1;
  std::vector<unsigned> allcover(n,1);
  for(i=0; i<allcover.size(); i++)
  {
    counter++; std::cout<<counter<<". ";
    std::vector<unsigned> cover=allcover;
    cover[i]=0;
    cover=procedure_1(neighbors,cover);
    s=cover_size(cover);
    if(s<min) min=s;
    //for(j=0; j<n; j++)
    //cover=procedure_2(neighbors,cover,j);
    //s=cover_size(cover);
    //if(s<min) min=s;
    covers.push_back(cover);
  }
  //Pairwise Unions
  for(p=0; p<covers.size()-1; p++)
  {
    for(q=p+1; q<covers.size(); q++)
    {
      counter++; std::cout<<counter<<". ";
      std::vector<unsigned> cover=allcover;
      for(r=0; r<cover.size(); r++)
        if(covers[p][r]==0 && covers[q][r]==0) cover[r]=0;
      cover=procedure_1(neighbors,cover);
      s=cover_size(cover);
      if(s<min) min=s;
      //for(j=0; j<k; j++)
      //cover=procedure_2(neighbors,cover,j);
      //s=cover_size(cover);
      //if(s<min) min=s;
      std::cout<<"Vertex Cover ("<<s<<"): ";
      for(j=0; j<cover.size(); j++) if(cover[j]==1) std::cout<<j+1<<" ";
      std::cout<<"\n";
      std::cout<<"Vertex Cover Size: "<<s<<"\n";
    }
  }
  std::cout <<"Minimum Vertex Cover size found is "<<min<<"."<<"\n";
}

int DPForWMVC(std::vector<unsigned>& x, int i, int sum,
    const std::vector<unsigned>& CG,
    const std::vector<unsigned>& range,
    int& best_so_far)
{
  if (sum >= best_so_far)
    return INT_MAX;
  //double runtime = (double) (clock() - start_time) / CLOCKS_PER_SEC;
  //if (runtime > time_limit)
    //return -1; // run out of time
  else if (i == (int) x.size())
  {
    best_so_far = sum;
    return sum;
  }
  else if (range[i] == 0) // vertex i does not have any edges.
  {
    int rst = DPForWMVC(x, i + 1, sum, CG, range, best_so_far);
    if (rst < best_so_far)
    {
      best_so_far = rst;
    }
    return best_so_far;
  }

  int cols = x.size();

  // find minimum cost for this vertex
  int min_cost = 0;
  for (int j = 0; j < i; j++)
  {
    if (min_cost + x[j] < CG[j * cols + i]) // infeasible assignment
    {
      min_cost = CG[j * cols + i] - x[j]; // cost should be at least CG[i][j] - x[j];
    }
  }


  int best_cost = -1;
  const unsigned inc(1); // Increment value (could we use a larger value?)
  for (int cost = min_cost; cost <= range[i]; cost+=inc)
  {
    x[i] = cost;
    int rst = DPForWMVC(x, i + 1, sum + x[i], CG, range, best_so_far);
    if (rst < best_so_far)
    {
      best_so_far = rst;
      best_cost = cost;
    }
  }
  if (best_cost >= 0)
  {
    x[i] = best_cost;
  }

  return best_so_far;
}


unsigned weightedVertexCover(const std::vector<unsigned>& CG)
{
  unsigned num_of_agents(sqrt(CG.size()));
  unsigned rst = 0;
  std::vector<bool> done(num_of_agents, false);
  for (unsigned i = 0; i < num_of_agents; i++)
  {
    if (done[i])
      continue;
    std::vector<unsigned> range;
    std::vector<unsigned> indices;
    range.reserve(num_of_agents);
    indices.reserve(num_of_agents);
    unsigned num = 0;
    std::queue<unsigned> Q;
    Q.push(i);
    done[i] = true;
    while (!Q.empty())
    {
      unsigned j = Q.front(); Q.pop();
      range.push_back(0);
      indices.push_back(j);
      for (unsigned k = 0; k < num_of_agents; k++)
      {
        if (CG[j * num_of_agents + k] > 0)
        {
          range[num] = std::max(range[num], CG[j * num_of_agents + k]);
          if (!done[k])
          {
            Q.push(k);
            done[k] = true;
          }
        }
        else if (CG[k * num_of_agents + j] > 0)
        {
          range[num] = std::max(range[num], CG[k * num_of_agents + j]);
          if (!done[k])
          {
            Q.push(k);
            done[k] = true;
          }
        }
      }
      num++;
    }
    if (num == 1) // no edges
      continue;
    else if (num == 2) // only one edge
    {
      rst += std::max(CG[indices[0] * num_of_agents + indices[1]], CG[indices[1] * num_of_agents + indices[0]]); // add edge weight
      continue;
    }
    std::vector<unsigned> G(num * num, 0);
    for (unsigned j = 0; j < num; j++)
    {
      for (unsigned k = j + 1; k < num; k++)
      {
        G[j * num + k] = std::max(CG[indices[j] * num_of_agents + indices[k]], CG[indices[k] * num_of_agents + indices[j]]);
      }
    }
    //if (num > ILP_node_threshold) // solve by ILP
    //{
      //rst += ILPForWMVC(G, range);
    //}
    //else // solve by dynamic programming
    //{
      std::vector<unsigned> x(num);
      int best_so_far = INT_MAX;
      rst += DPForWMVC(x, 0, 0, G, range, best_so_far);
    //}
    //double runtime = (double) (clock() - start_time) / CLOCKS_PER_SEC;
    //if (runtime > time_limit)
      //return -1; // run out of time
  }

  //test
  /*std::vector<unsigned> x(N, 0);
    std::vector<unsigned> range(N, 0);
    for (unsigned i = 0; i < N; i++)
    {
    for (unsigned j = i + 1; j < N; j++)
    {
    range[i] = std::max(range[i], CG[i * N + j]);
    range[j] = std::max(range[j], CG[i * N + j]);
    }
    }
    unsigned best_so_far = INT_MAX;
    unsigned rst2 = DPForWMVC(x, 0, 0, CG, range, best_so_far);
    if ( rst != rst2)
    std::cout << "ERROR" <<std::endl;*/

  return rst;
}

#endif
