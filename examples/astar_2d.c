// Sample implementation of A* search on a simple 2D Cartesian space
//
// v1.0   2007-10-06, Chaos of Lost Souls MUD, http://lostsouls.org/
// v1.1   2013-08-18, Chaos; graph terminology update

#include <astar.h>

inherit "/mod/algorithm/astar";

#define Map_Min_X   1
#define Map_Max_X   20
#define Map_Min_Y   1
#define Map_Max_Y   10
#define X           0
#define Y           1

// Neighbors rule produces nodes and edges for the points adjacent to us.

mixed * astar_neighbors_rule(mixed * pathfind) {
	int * node = pathfind[Astar_Pathfind_Active_Node];
	mixed * out = ({});
	if(node[X] > Map_Min_X)
		out += ({({
			({ node[X] - 1, node[Y] }),     // Adjacent node
			({ -1, 0 }),                    // Edge to adjacent node
			1,                              // Cost of edge
		})});
	if(node[X] < Map_Max_X)
		out += ({({
			({ node[X] + 1, node[Y] }),     // Adjacent node
			({ 1, 0 }),                     // Edge to adjacent node
			1,                              // Cost of edge
		})});
	if(node[Y] > Map_Min_Y)
		out += ({({
			({ node[X], node[Y] - 1 }),     // Adjacent node
			({ 0, -1 }),                    // Edge to adjacent node
			1,                              // Cost of edge
		})});
	if(node[Y] < Map_Max_Y)
		out += ({({
			({ node[X], node[Y] + 1 }),     // Adjacent node
			({ 0, 1 }),                     // Edge to adjacent node
			1,                              // Cost of edge
		})});
	return out;
}

// Distance rule calculates the distance from a point to the target
// node.  Using the Pythagorean Theorem, yo.

float astar_distance_rule(mixed * pathfind) {
	int * a = pathfind[Astar_Pathfind_Active_Node];
	int * b = pathfind[Astar_Pathfind_To];
	int dx = a[X] - b[X];
	int dy = a[Y] - b[Y];
	return sqrt(dx * dx + dy * dy);
}

// Node key rule gives us a unique value corresponding to a node that
// can be reliably used as a mapping key.  Since we're using arbitrary
// two-element arrays as nodes, and arrays are pointers that will give
// unpredictable behavior when used as mapping keys, we need to use this
// so the algorithm can tell where it's been.

int astar_node_key_rule(int * node) {
	return (node[X] << 16) | node[Y];
}

// Run limit rule tells the algorithm when to stop running and continue
// on a call_out.

int astar_run_limit_rule(mixed * pathfind) {
	return get_eval_cost() < __MAX_EVAL_COST__ / 2;
}

// Set up the A* module.

void create() {
	set_astar_neighbors_rule(#'astar_neighbors_rule);
	set_astar_distance_rule(#'astar_distance_rule);
	set_astar_node_key_rule(#'astar_node_key_rule);
	set_astar_run_limit_rule(#'astar_run_limit_rule);
	set_astar_caching(1);
}

// This is our callback when pathfinding completes (for better or worse);
// it displays the path to this_player().

void end_random_pathfind(mixed * path) {
	if(path) {
		mixed * nodes = path[Astar_Path_Nodes];
		write("Path from " + nodes[0][X] + "," + nodes[0][Y] + " to " +
			nodes[<1][X] + "," + nodes[<1][Y] + ":\n");
		foreach(mixed * node : nodes)
			write("    " + node[X] + "," + node[Y] + "\n");
	} else {
		write("Cannot find path.\n");
	}
}

// Picks two random points in our coordinate space and pathfinds between
// them.

void start_random_pathfind() {
	int * start = ({ Map_Min_X + random(Map_Max_X - Map_Min_X + 1),
		Map_Min_Y + random(Map_Max_Y - Map_Min_Y + 1) });
	int * target = ({ Map_Min_X + random(Map_Max_X - Map_Min_X + 1),
		Map_Min_Y + random(Map_Max_Y - Map_Min_Y + 1) });
	astar_find_path(start, target, 0, #'end_random_pathfind);
}
