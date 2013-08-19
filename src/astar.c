// A* Search Algorithm Module
//
// Provides a fairly general implementation of the A* search algorithm
// that can be used to search arbitrary problem spaces.  A* search is a
// modification of best-first search that uses a heuristic to determine
// which paths to preferentially search, frequently based on coordinate-space
// distance from the path endpoint to the goal.  It is commonly regarded as
// the best general algorithm for pathfinding within a Cartesian space.
// See http://en.wikipedia.org/wiki/A*_search_algorithm for more.
//
// This module's official home: http://lostsouls.org/grimoire_astar
// New versions will be posted there.
//
// You may do anything you like with this code so long as these two lines,
// the thirteen preceding and the twenty-two following are retained intact.
// 
// v1.0    2007-05-11, Chaos of Lost Souls, http://lostsouls.org/
// v1.1    2007-05-29, Chaos; factored astar.h out of astar.c
// v1.2    2007-06-13, Chaos; moved all state to pathfind data structure
// v1.3    2007-07-09, Chaos; added edge costs, code cleanup
// v1.4    2007-08-12, Chaos; added active edge to pathfind structure
// v1.4.1  2007-08-31, Chaos; improved comments
// v1.5    2007-09-03, Chaos; made node not considered visited until valid
// v1.5.1  2007-09-04, Chaos: improved comments
// v1.5.2  2007-10-06, Chaos: moved sample implementation into its own file
// v1.5.3  2007-10-06, Chaos: minor comment cleanup
// v1.5.4  2008-12-29, Chaos: run tracking improvements, formatting >80 col
// v1.5.5  2009-04-26, Chaos: single-argument 'extra', pathfind as callback
//   argument, pathfind as astar_find_path() return value, restructuring
// v1.5.6  2009-06-30, Chaos: improved cache implementation
// v1.5.7  2009-07-29, Chaos: reordered validate key calculations
// v1.5.8  2009-10-02, Chaos: refactoring and cleanup
// v1.5.9  2009-10-13, Chaos: cache refactoring and tweaking
// v1.5.10 2009-10-26, Chaos: LDMud 3.3 warning elision
// v1.6    2011-01-28, Chaos: added pathfind no-continue flag, flags in start,
//   scheduling rule support allowing usage of call_out() to be overridden
// v1.7    2013-08-18, Chaos: cohered terminology with standard graph theory

// This module is written for LPC as implemented by the LDMud driver,
// http://www.bearnip.com/lars/proj/ldmud.html.  An effort has been made
// to avoid highly driver-specific features, and the module should be
// portable to e.g. MudOS with moderate effort.

// This is not a toy or demonstration version of A* search; it is a module
// written to be used (and in active use) in a production environment that
// needs to deploy A* in a variety of contexts.  I've worked to make it
// clear and well-documented, but its primary purpose is not to explain A*
// search, it is to provide a clear, well-documented module that implements
// a working practical version of the algorithm.  In Lost Souls, the module
// is presently used to perform pathfinding in 2D and 3D coordinate-space
// and hybrid coordinate-space/arbitrary-linkage MUD areas, as well as
// searching in the problem space made up of A* results from those instances.

// In order to work properly with A* search as implemented by this module,
// your situation needs to be describable in terms of a few crucial concepts.
//
// The first concept is nodes: there need to be things resembling locations,
// states, options, or the like that can be represented in some consistent
// fashion.  One common kind of node is a point in a Cartesian coordinate
// space, an X, Y or X, Y, Z location; another is a MUD room filename.  The
// concept of a node isn't limited to describing location; nodes could as
// easily be behaviors, frames of mind, geometrical arrangements, goals, or
// anything else.  You should be able to generally figure out some sort of
// distance (more generally, a cost-to-reach estimate) between two nodes
// without any other information; a Cartesian space works well for this part.
// It's okay if there are some nodes you can find a distance for and some 
// you can't, as with a MUD area that has some rooms on a Cartesian grid and
// some off of it -- this module is set up to deal with that -- but if you 
// can't find a distance for anything and all of your nodes have the same
// cost to move between them, then you're not getting any benefit out of
// the A* algorithm and may as well just do an exhaustive search.
//
// The second concept is 'edges': there need to be quantifiable ways to
// move between adjacent nodes.  These might be simply lists of offsets in
// a coordinate space, e.g. ({ 0, 1, 0 }), they may be directions like
// "north" or commands like "enter portal", or for more esoteric nodes they
// may be abstract link identifiers or other ways of describing change.
//
// The third concept is 'costs'.  Edges have an associated cost; the lower
// the cost of the edge, the more desirable it is to use it.  This may
// represent relative difficulty of terrain, varying time delays between
// exits, or anything else that affects the desirability of using a edge.
// If all edges are equally desirable, just use 1 for their cost.
//
// The information you *must* be able to provide the algorithm in order for
// it to function is the answer to the question "which nodes are adjacent
// to this node and how do I get to them from here?"  Everything else is
// optional.
//
// Take a look at the file astar_2d_example.c for a sample implementation.

// This module uses macros from the file astar.h, which should be found
// accompanying this code, for its 'path' and 'pathfind' data structures
// and its result codes.  astar.h should be placed in a suitable include
// directory.

#include <astar.h>

// SECTION: Instance configuration
//
// These configuration functions are used by an application of the astar
// module to define its behavior and allow it to retrieve the information
// it works with.  Most of them are passed a closure (function pointer)
// pointing to a function you have defined, which the astar module then
// calls when it needs to.  Check the sample implementation in
// astar_2d_example.c for how it looks in practice.

// Neighbors rule
//
// The neighbors rule is used to retrieve all nodes adjacent to a specified
// node and the edges used to reach them.  It is called with an astar
// pathfind data structure (as defined by the Astar_Pathfind_* macros in
// astar.h) as argument; this data structure is the same one used by the
// algorithm for tracking the pathfinding attempt in general, which means
// that 1) the fields in it all contain valid information and can be used
// to examine the pathfind, and 2) the fields in it should not be changed
// or you will almost certainly cause errors.  Some fields in the data
// structure will be set specifically for this retrieval:
//
//     pathfind[Astar_Pathfind_Active_Node]
//         Will be set to the node whose neighbors we want to retrieve.
//     pathfind[Astar_Pathfind_Active_Edge]
//         Will be set to the edge from the previous node in the path
//         to the active node.  Set to 0 at the beginning of a path.
//     pathfind[Astar_Pathfind_Active_Path]
//         Will be set to the entire path leading to the active node.
//         This is a path data structure as defined by the Astar_Path_*
//         macros in astar.h.
//
// The return value that the neighbors rule should provide is an array of
// three-element arrays in which the first element is a node, the second
// element is the edge used to reach it, and the third element is the cost
// of the edge.  That means something like this:
//
//     return ({({
//         FIRST_NODE,
//         FIRST_EDGE,
//         COST_OF_FIRST_EDGE,
//     }), ({
//         SECOND_NODE,
//         SECOND_EDGE,
//         COST_OF_SECOND_EDGE,
//     })});
//
// Alternately, the neighbors rule may return Astar_Result_Processing,
// which tells the algorithm to retry the request slightly later via the
// scheduling rule.
//
// A neighbors rule is the only one that absolutely must be defined in order
// to allow the algorithm to function.

private closure neighbors_rule;

void set_astar_neighbors_rule(closure val) {
	neighbors_rule = val;
}

closure query_astar_neighbors_rule() {
	return neighbors_rule;
}

// Distance rule
//
// The distance rule is used to determine the distance (for non-locational
// problem spaces, think of it as a cost estimate) from a specified node
// to the target node.  It is called with the astar pathfind data structure
// structure; see the notes on the neighbors rule for more on this.  Fields
// of particular concern to the distance rule are:
//
//     pathfind[Astar_Pathfind_To]
//         The destination node of the pathfind; we are trying to find
//         the distance from the active node to here.
//     pathfind[Astar_Pathfind_Active_Node]
//         The node whose distance from the destination node we want to
//         know.
//
// The return value needed is a int or float value indicating the distance
// (or estimating the cost), or -1 if the distance cannot be determined.
//
// Using a distance rule is optional but strongly encouraged.

private closure distance_rule;

void set_astar_distance_rule(closure val) {
	distance_rule = val;
}

closure query_astar_distance_rule() {
	return distance_rule;
}

// Node rule
//
// The node rule is used to convert the representation of a node into the
// form you want.  If, for instance, the pathfinder routine might wind up
// called with a string filename or an object for its nodes, and you want to
// use the filename for how you're going to work with nodes internally, you
// can define this hook in order to perform the conversion.  It is called
// with a node representation as its argument; this is provided by the code
// requesting the pathfind and can be anything at all.  The return value
// needed is the final desired representation for the node.

private closure node_rule;

void set_astar_node_rule(closure val) {
	node_rule = val;
}

closure query_astar_node_rule() {
	return node_rule;
}

// Node key rule
//
// The node key rule is used to represent a node for purposes of checking
// whether it has been visited.  You will usually need to use this if your
// nodes are normally represented as pointers (arrays or mappings), unless
// the pointers for a given node can be guaranteed to always be the same.
// Otherwise, the system can't tell which nodes it's visited.  It is called
// with a node as argument.  The return value needed is the "flat"
// representation to use for the node, most typically a string or int.

private closure node_key_rule;

void set_astar_node_key_rule(closure val) {
	node_key_rule = val;
}

closure query_astar_node_key_rule() {
	return node_key_rule;
}

// Completion rule
//
// The completion rule can be used to determine whether an acceptable path
// destination has been found; if one is not supplied, equivalency between
// the node keys of the prospective completion node and the target node
// is checked.  It is called with the astar pathfind data structure as
// argument; see the notes on the neighbor rule for more about this.  Fields
// of particular relevance to this rule are:
//
//     pathfind[Astar_Pathfind_To]
//         The destination node of the pathfind.
//     pathfind[Astar_Pathfind_Active_Node]
//         The node being checked.
//
// It should return true if the node is a valid completion node.

private closure completion_rule;

void set_astar_completion_rule(closure val) {
	completion_rule = val;
}

closure query_astar_completion_rule() {
	return completion_rule;
}

// Cycle process
//
// The cycle process is executed at the beginning of every pathfinding
// 'cycle'.  A new cycle is begun when the pathfind starts and when it
// continues via the scheduling rule, if applicable.  It is called with
// the astar pathfind data structure as argument; see the notes on the
// neighbor rule for more about this.  Fields of particular relevance to
// this rule are:
//
//     pathfind[Astar_Pathfind_Cycle_Start]
//         The utime() when the current pathfinding cycle began.
//     pathfind[Astar_Pathfind_Cycle_Index]
//         The serial number of the current pathfinding cycle.
//
// Its return value is ignored.

private closure cycle_process;

void set_astar_cycle_process(closure val) {
	cycle_process = val;
}

closure query_astar_cycle_process() {
	return cycle_process;
}

// Run limit rule
//
// The run limit rule is used to check whether the algorithm has run for too
// long and should be rescheduled or aborted.  It is called with the astar
// pathfind data structure as argument.  The return value needed is any true
// value if the run limit has been exceeded, false if not.

private closure run_limit_rule;

void set_astar_run_limit_rule(closure val) {
	run_limit_rule = val;
}

closure query_astar_run_limit_rule() {
	return run_limit_rule;
}

// Caching
//
// The cache retains the pathfinding results that have been obtained so they
// do not have to be recalculated after being requested once, at the cost of
// some memory usage.  One would use set_astar_caching(1) in the instance to
// turn on caching.
//
// If your neighbors rule does not always return the same results for a given
// node, for instance if you use extra arguments to provide varying results
// as with terrain difficulty that varies by the person traversing it, then
// you should generally not turn on caching, because the cached paths may not
// remain valid and could be returned in inappropriate circumstances.

private mapping cache;

void set_astar_caching(int val) {
	if(val)
		cache = ([]);
	else
		cache = 0;
}

int query_astar_caching() {
	return cache && 1;
}

mapping query_astar_cache() {
	return cache;
}

// Validate key rule
//
// The validate key rule is only meaningful if you have caching turned on.
// It can be used to provide mapping-key-usable representations of closures
// passed to astar_find_path() as its 'validate' argument.  A common way to
// construct the rule is to return the to_string() of the closure; this
// implies that (and should only be done if) a particular 'validate' closure
// will always return the same way for a particular node.  This is used for
// caching pathfinding results with 'validate' rules.  If you cannot guarantee
// consistent results for a given 'validate' function, you should return 0
// from the validate key rule so that the system knows not to cache the path
// using it.  If this rule is not defined, paths generated with 'validate'
// rules will not be cached.  The validate key rule is called with the astar
// pathfind data structure as argument, with the most relevant field being:
//
//     pathfind[Astar_Pathfind_Validate]
//         The 'validate' closure provided for the pathfinding attempt
// 
// The validate key rule should return the representation to use for the
// 'validate' closure -- usually a string or int, or 0 if no representation is
// appropriate or available.

private closure validate_key_rule;

void set_astar_validate_key_rule(closure val) {
	validate_key_rule = val;
}

closure query_astar_validate_key_rule() {
	return validate_key_rule;
}

// Scheduling rule
//
// The astar module uses its scheduling rule to request a future call of a
// function, for purposes of continuing pathfinding that cannot be completed
// in the current execution because of a run limit rule or other conditions.
// The rule is passed arguments of 1) the function that should be called
// 2) approximately how many seconds later it should be called 3) an astar
// pathfind data structure that should be passed as the argument to the
// function called.  The default scheduling rule is #'call_out, i.e. the
// purpose of this rule is to allow you to override the module's default usage
// of call_out() for scheduling purposes.

private closure scheduling_rule;

void set_astar_scheduling_rule(closure val) {
	scheduling_rule = val;
}

closure query_astar_scheduling_rule() {
	return scheduling_rule;
}

// SECTION: Internal support functions
//
// These are functions used by the A* module.  Instances do not need to
// interact with them.

// astar_path_sort()
//
// Sorting function for paths, based on their cost.  Sorts low-cost paths
// to the end of the list.
//
// Portability note: this function provides return values suitable for a
// quicksort-style sorting algorithm; porting to drivers that implement
// sort_array() as other algorithms will require changes to this function.
// The version of this function that would be appropriate to the original
// LPMud sort_array(), which expects only 0 or 1 as return value, would be:
// 
// private int astar_path_sort(mixed * a, mixed * b) {
//     return a[Astar_Path_Cost] < b[Astar_Path_Cost];
// }

private int astar_path_sort(mixed * a, mixed * b) {
	float cost_a = a[Astar_Path_Cost];
	float cost_b = b[Astar_Path_Cost];
	if(cost_a > cost_b)
		return -1;
	else if(cost_a < cost_b)
		return 1;
	else
		return 0;
}

// astar_distance()
//
// Distance retrieval process.  The distance rule is allowed to return -1
// if, for whatever reason, it doesn't know how far it is between nodes;
// the cost of the new path is set to be one greater than the cost of the
// path it extends.

private float astar_distance(mixed * pathfind) {
	if(!distance_rule)
		return pathfind[Astar_Pathfind_Active_Path][Astar_Path_Distance] + 1.0;
	mixed res = funcall(distance_rule, pathfind);
	if(res == -1)
		return pathfind[Astar_Pathfind_Active_Path][Astar_Path_Distance] + 1.0;
	else
		return res;
}

// astar_key()
//
// Node key retrieval process.  Finds the representation to use for the
// node in checking visited status.

private mixed astar_key(mixed node) {
	return node_key_rule ? funcall(node_key_rule, node) : node;
}

// astar_cached_path()
//
// Cached path retrieval.

private mixed astar_cached_path(mixed * pathfind) {
	if(!cache)
		return 0;
	closure validate = pathfind[Astar_Pathfind_Validate];
	mixed validate_key = validate && validate_key_rule && funcall(validate_key_rule, pathfind);
	if(validate && !validate_key)
		return 0;
	mapping validate_cache = cache[validate_key];
	if(!validate_cache)
		return 0;
	mapping from_cache = validate_cache[astar_key(pathfind[Astar_Pathfind_From])];
	if(!from_cache)
		return 0;
	mixed entry = from_cache[astar_key(pathfind[Astar_Pathfind_To])];
	if(!entry)
		return 0;
	entry[Astar_Cache_Hits]++;
	entry[Astar_Cache_Timestamp] = time();
	return entry;
}

// astar_pathfind_done()
//
// Internal function for handling the end of a pathfind.

private void astar_pathfind_done(mixed * pathfind, mixed result) {
	pathfind[Astar_Pathfind_Result] = result;
	if(pathfind[Astar_Pathfind_Callback] && !(pathfind[Astar_Pathfind_Control_Flags] & Astar_Pathfind_Control_Flag_Silent))
		funcall(pathfind[Astar_Pathfind_Callback], pathfind);
}

// astar_pathfind_close()
//
// Internal function for handling the end of a pathfind.

private void astar_pathfind_close(mixed * pathfind, mixed result) {
	if(cache && !(pathfind[Astar_Pathfind_Control_Flags] & Astar_Pathfind_Control_Flag_Uncache)) {
		// Calculate validate key beforehand in case the callback changes anything that interferes with generating it
		closure validate = pathfind[Astar_Pathfind_Validate];
		mixed validate_key = validate && validate_key_rule && funcall(validate_key_rule, pathfind);
		astar_pathfind_done(pathfind, result);
		if((validate_key || !validate) && !(pathfind[Astar_Pathfind_Control_Flags] & Astar_Pathfind_Control_Flag_Uncache)) {
			mapping validate_cache = cache[validate_key] ||= ([]);
			mixed from_key = astar_key(pathfind[Astar_Pathfind_From]);
			mapping from_cache = validate_cache[from_key] ||= ([]);
			mixed to_key = astar_key(pathfind[Astar_Pathfind_To]);
			mixed entry = allocate(Astar_Cache_Fields);
			entry[Astar_Cache_Path] = pointerp(result) && result;
			entry[Astar_Cache_Timestamp] = time();
			from_cache[to_key] = entry;
		}
	} else {
		astar_pathfind_done(pathfind, result);
	}
}

// astar_pathfinder()
//
// Performs the actual work of path calculation; takes a fully
// initialized (and maybe partially processed) pathfind data structure
// as argument.  Can resume from any point in the pathfinding
// process, which is what lets the algorithm suspend activity when
// a run limit is reached and resume via the scheduling rule.

private void astar_pathfinder(mixed * pathfind) {
	if(cycle_process)
		funcall(cycle_process, pathfind);
	if(pathfind[Astar_Pathfind_Cycle_Index]) {
		if(pathfind[Astar_Pathfind_Control_Flags] & Astar_Pathfind_Control_Flag_Terminate)
			return astar_pathfind_done(pathfind, Astar_Result_Terminated);
		if(!(pathfind[Astar_Pathfind_Control_Flags] & Astar_Pathfind_Control_Flag_Uncache)) {
			mixed path = astar_cached_path(pathfind);
			if(path)
				return astar_pathfind_done(pathfind, path[Astar_Cache_Path] || Astar_Result_Impossible);
		}
	}
	pathfind[Astar_Pathfind_Cycle_Start] = utime();
	pathfind[Astar_Pathfind_Cycle_Index]++;
	pathfind[Astar_Pathfind_Cycle_Iterations] = 0;
	mixed to_key = astar_key(pathfind[Astar_Pathfind_To]);
	for(;;) {
		pathfind[Astar_Pathfind_Cycle_Iterations]++;
		if(run_limit_rule && funcall(run_limit_rule, pathfind)) {
			if(pathfind[Astar_Pathfind_Callback] && !(pathfind[Astar_Pathfind_Control_Flags] & Astar_Pathfind_Control_Flag_No_Continue)) {
				pathfind[Astar_Pathfind_Cycle_Index]++;
				pathfind[Astar_Pathfind_Result] = Astar_Result_Processing;
				funcall(scheduling_rule, #'astar_pathfinder, 2, pathfind);
			} else {
				pathfind[Astar_Pathfind_Result] = Astar_Result_Cut_Off;
			}
			return;
		}
		mixed * extended = ({});
		// Sort the paths on their cost; we only want to deal with the lowest-cost paths out of the ones we have.
		mixed * paths = sort_array(pathfind[Astar_Pathfind_Paths], #'astar_path_sort);
		// Get the cost of the best path on hand; we only want paths with this cost.
		float cost = paths[<1][Astar_Path_Cost];
		int ix;
		mixed * final = 0;
		// Check for possible extensions on as many paths as we have that are at our best cost.
		for(ix = sizeof(paths) - 1; ix >= 0 && paths[ix][Astar_Path_Cost] <= cost; ix--) {
			mixed * path = paths[ix];
			pathfind[Astar_Pathfind_Active_Path] = path;
			pathfind[Astar_Pathfind_Active_Node] = path[Astar_Path_Nodes][<1];
			mixed * edges = path[Astar_Path_Edges];
			pathfind[Astar_Pathfind_Active_Edge] = sizeof(edges) ? edges[<1] : 0;
			// Retrieve the list of neighbor nodes and edges to reach them.
			mixed neighbors = funcall(neighbors_rule, pathfind);
			if(!pointerp(neighbors)) {
				if(neighbors == Astar_Result_Processing) {
					if(pathfind[Astar_Pathfind_Callback] && !(pathfind[Astar_Pathfind_Control_Flags] & Astar_Pathfind_Control_Flag_No_Continue)) {
						pathfind[Astar_Pathfind_Cycle_Index]++;
						pathfind[Astar_Pathfind_Result] = Astar_Result_Processing;
						funcall(scheduling_rule, #'astar_pathfinder, 2, pathfind);
					} else {
						pathfind[Astar_Pathfind_Result] = Astar_Result_Cannot_Continue;
					}
					return;
				} else {
					raise_error("Invalid return value from neighbors rule");
				}
			}
			foreach(mixed * neighbor : neighbors) {
				mixed node = neighbor[0];
				mixed edge = neighbor[1];
				mixed ncost = neighbor[2];
				// If we've already been here, never mind.
				mixed key = astar_key(node);
				if(pathfind[Astar_Pathfind_Visited][key])
					continue;
				// Register node and edge in pathfind structure.
				pathfind[Astar_Pathfind_Active_Node] = node;
				pathfind[Astar_Pathfind_Active_Edge] = edge;
				// If we have a validation rule for nodes, check against it.
				if(pathfind[Astar_Pathfind_Validate] && !funcall(pathfind[Astar_Pathfind_Validate], pathfind))
					continue;
				// Okay, then, now we've been here.
				pathfind[Astar_Pathfind_Visited][key] = 1;
				// This is now a valid extension.  Assemble the extended path with this node added to it, and calculate the
				// distance and cost.
				mixed * active_path = pathfind[Astar_Pathfind_Active_Path];
				mixed * ext_path = copy(active_path);
				ext_path[Astar_Path_Nodes] += ({ node });
				ext_path[Astar_Path_Edges] += ({ edge });
				ext_path[Astar_Path_Distance] = astar_distance(pathfind);
				// The cost of the extended path is its distance from the target node, plus the portion of the base path's cost
				// that is not based on its distance, plus the cost of the edge.
				ext_path[Astar_Path_Cost] = active_path[Astar_Path_Cost] - active_path[Astar_Path_Distance] + ext_path[Astar_Path_Distance] + ncost;
				// If the node we just reached is the target, add this path to the list of final paths and stop tracking
				// path extensions; otherwise, track path extension, if they are being tracked.
				if(completion_rule ? funcall(completion_rule, pathfind) : (key == to_key)) {
					final ||= ({});
					final += ({ ext_path });
					extended = 0;
				} else if(extended) {
					extended += ({ ext_path });
				}
			}
		}
		// If we have any final paths, choose the best one as our result and finish.
		if(final) {
			if(sizeof(final) > 1)
				final = sort_array(final, #'astar_path_sort);
			return astar_pathfind_close(pathfind, final[0]);
		}
		// Get rid of the paths that we extended, and add the newly extended paths to our list.
		paths = paths[0 .. ix] + extended;
		// If we no longer have any paths to examine, we're out of luck.
		if(!sizeof(paths))
			return astar_pathfind_close(pathfind, Astar_Result_Impossible);
		pathfind[Astar_Pathfind_Paths] = paths;
		if(pathfind[Astar_Pathfind_Control_Flags] & Astar_Pathfind_Control_Flag_Terminate)
			return astar_pathfind_done(pathfind, Astar_Result_Terminated);
	}
}

// SECTION: Operational interface

// astar_find_path()
//
// Performs pathfinding starting with the 'from' node, searching for the
// 'to' node.
//
// 'validate' can be used to provide a closure that checks whether a node
// is valid to include in the path.  It is called with the astar pathfind
// data structure as argument; see the notes on the neighbor rule for more
// on this.  Fields of particular concern for the validation call are:
// 
//     pathfind[Astar_Pathfind_Active_Node]
//         Set to the node being examined.
//     pathfind[Astar_Pathfind_Active_Edge]
//         The edge to reach the active node from the previous node.
//     pathfind[Astar_Pathfind_Active_Path]
//         The entire path leading to the previous node; this is a path
//         data structure as defined by the Astar_Path_* macros in astar.h.
//
// The 'validate' function should return true if the node being examined
// should be included in the path.
//
// 'callback' can be used to provide a closure to be called at the completion
// of pathfinding.  It will be called with the astar pathfind data structure
// as argument; the most relevant field is:
//
//     pathfind[Astar_Pathfind_Result]
//         The final result of the pathfind; an astar path data structure
//         (from astar.h) if pathfinding was successful, or an Astar_Result_*
//         code (also from astar.h) for their respective conditions.
//
// Providing a callback allows pathfinding to be carried out in parts via
// the scheduling rule; if no callback is provided, or if the control flag
// Astar_Control_Flag_No_Continue is used, then pathfinding will execute
// until it reaches the run limit defined by the instance, if any, or it hits
// the driver eval limit.  With a callback and the control flag not in use,
// pathfinding runs until it reaches the run limit, then continues via the
// scheduling rule, normally two seconds later, continuing in this fashion
// until a path is found, all possible paths have been searched, or some other
// condition terminates the pathfind.
//
// The fifth argument is an integer that may contain control flags, as defined
// by the Astar_Pathfind_Control_Flag_* macros in astar.h.
//
// The sixth argument is an arbitrary user-supplied value.  It will be
// passed to 'callback' after the path argument, and is accessible as
// Astar_Pathfind_Extra in the pathfind data structure.
//
// The return value is the astar pathfind data structure (from astar.h) that
// defines the pathfind request.  It can be manipulated (for example, by doing
// pathfind[Astar_Pathfind_Control_Flags] |= Astar_Pathfind_Control_Flag_Terminate)
// to alter or inspect the pathfind request.  If pathfind[Astar_Pathfind_Result]
// is anything other than Astar_Result_Processing, the pathfind request has
// completed.

varargs mixed * astar_find_path(mixed from, mixed to, closure validate, closure callback, int control_flags, mixed extra) {
	// Constrain our representation of our 'from' and 'to' nodes
	if(node_rule) {
		from = funcall(node_rule, from);
		to = funcall(node_rule, to);
	}
	// Default scheduling rule to #'call_out if none has been set
	scheduling_rule ||= #'call_out;
	// Set up pathfinding data structure
	mixed * pathfind = allocate(Astar_Pathfind_Fields);
	pathfind[Astar_Pathfind_From] = from;
	pathfind[Astar_Pathfind_To] = to;
	pathfind[Astar_Pathfind_Validate] = validate;
	pathfind[Astar_Pathfind_Callback] = callback;
	pathfind[Astar_Pathfind_Extra] = extra;
	pathfind[Astar_Pathfind_Start_Time] = utime();
	pathfind[Astar_Pathfind_Cycle_Index] = 0;
	pathfind[Astar_Pathfind_Active_Node] = from;
	pathfind[Astar_Pathfind_Active_Edge] = 0;
	pathfind[Astar_Pathfind_Control_Flags] = control_flags;
	// Check for a cached path
	mixed path = astar_cached_path(pathfind);
	if(path) {
		pathfind[Astar_Pathfind_Result] = path[Astar_Cache_Path] || Astar_Result_Impossible;
		if(callback)
			funcall(callback, pathfind);
		return pathfind;
	}
	// Set up our starting point based on the 'from' node
	mixed * start = allocate(Astar_Path_Fields);
	pathfind[Astar_Pathfind_Active_Path] = start;
	start[Astar_Path_Nodes] = ({ from });
	start[Astar_Path_Edges] = ({});
	start[Astar_Path_Distance] = astar_distance(pathfind);
	start[Astar_Path_Cost] = start[Astar_Path_Distance];
	// Starting point becomes our path list, mapping to track where we've visited starts out populated with starting node
	pathfind[Astar_Pathfind_Paths] = ({ start });
	pathfind[Astar_Pathfind_Visited] = ([
		astar_key(from) : 1,
	]);
	astar_pathfinder(pathfind);
	return pathfind;
}

// astar_clear_cache()
//
// Clears out the contents of the cache.  This can be useful for allowing
// caching in spaces that you otherwise couldn't use caching with because
// some sort of dynamicism in them (changing exits, shifting graph
// connectivity, etc.) would invalidate cached paths.  Using this, you can
// call astar_clear_cache() when changes occur, so that no outdated paths
// will be returned.

void astar_clear_cache() {
	if(!cache)
		raise_error("astar_clear_cache() called with caching off");
	cache = ([]);
}

// astar_prune_cache()
//
// Prunes entries from the cache.  The optional argument, threshold,
// influences how aggressively the pruning occurs.  A cache entry will be
// dropped if its most recent hit was longer ago, in seconds, than
// 'threshold' plus the number of hits it has had times a tuning factor.
// 'threshold' defaults to Astar_Prune_Cache_Default_Threshold, 7200.
// The tuning factor is Astar_Prune_Cache_Hit_Factor, 60.
//
// It is generally appropriate to call this function from reset() of an
// object that inherits this module and uses caching.

varargs void astar_prune_cache(int threshold) {
	if(!cache)
		raise_error("astar_prune_cache() called with caching off");
	threshold ||= Astar_Prune_Cache_Default_Threshold;
	int cutoff = time() - threshold;
	foreach(string validate_key, mapping validate_cache : cache) {
		foreach(string from_key, mapping from_cache : validate_cache) {
			foreach(string to_key, mixed entry : from_cache)
				if(entry[Astar_Cache_Timestamp] + (entry[Astar_Cache_Hits] * Astar_Prune_Cache_Hit_Factor) < cutoff)
					map_delete(from_cache, to_key);
			if(!sizeof(from_cache))
				map_delete(validate_cache, from_key);
		}
		if(!sizeof(validate_cache))
			map_delete(cache, validate_key);
	}
}
