double search_costonly(AnyaNode start, AnyaNode goal)
{
  init();
  double cost = -1;
  if(!expander.validate_instance(start, goal))
  {
    return cost;
  }

  SearchNode startNode = generate(start);
  startNode.reset();
  open.insert(startNode, heuristic.getValue(start, goal), 0);

  while(!open.isEmpty())
  {
    SearchNode current = (SearchNode)open.removeMin();
    if(verbose) { std::cout << "expanding (f="+current.getKey( << "\n"+") "+current.toString()); }
    expander.expand(current.getData());
    expanded++;
    heap_ops++;
    if(current.getData().interval.contains(goal.root))
    {
      // found the goal
      cost = current.getKey();

      if(verbose)
      {
        print_path(current, System.err);
        System.err.println(goal.toString() + "; f=" + current.getKey());
      }
      break;
    }

    // unique id for the root of the parent node
    int p_hash = expander.hash(current.getData());

    // iterate over all neighbours
    while(expander.hasNext())
    {
      AnyaNode succ = expander.next();
      SearchNode neighbour = generate(succ);

      boolean insert = true;
      int root_hash = expander.hash(succ);
      SearchNode root_rep = roots_.get(root_hash);
      double new_g_value = current.getSecondaryKey() +
        expander.step_cost();

      // Root level pruning:
      // We prune a node if its g-value is larger than the best
      // distance to its root point. In the case that the g-value
      // is equal to the best known distance, we prune only if the
      // node isn't a sibling of the node with the best distance or
      // if the node with the best distance isn't the immediate parent
      if(root_rep != null)
      {
        double root_best_g = root_rep.getSecondaryKey();
        insert = (new_g_value - root_best_g)
          <= BitpackedGrid.epsilon;
        boolean eq = (new_g_value - root_best_g)
          >= -BitpackedGrid.epsilon;
        if(insert && eq)
        {
          int p_rep_hash = expander.hash(root_rep.parent.getData());
          insert = (root_hash == p_hash) || (p_rep_hash == p_hash);
        }
      }

      if(insert)
      {
        neighbour.reset();
        neighbour.parent = current;
        open.insert(neighbour,
            new_g_value +
            heuristic.getValue(neighbour.getData(), goal),
            new_g_value);
        roots_.put(root_hash, neighbour);

        if(verbose)
        {
          std::cout << "\tinserting with f=" + neighbour.getKey( << "\n" +" (g= "+new_g_value+");" + neighbour.toString());
        }

        heap_ops++;
        insertions++;
      }
      else
      {
        if(verbose)
        {
          std::cout << "\told rootg: "+root_rep.getSecondaryKey( << "\n");
          std::cout << "\tNOT inserting with f=" + neighbour.getKey( << "\n" +" (g= "+new_g_value+");" + neighbour.toString());
        }

      }
    }
  }
  if(verbose)
  {
    std::cout << "finishing search;" << "\n";
  }
  return cost;

  Path<AnyaNode> path = null;
  if(cost != -1)
  {
    SearchNode node = generate(target);
    do
    {
      path = new Path<AnyaNode>(node.getData(), path, node.getSecondaryKey());
      node = node.parent;

    }while(!(node.parent == null));
  }
  return path;
}


