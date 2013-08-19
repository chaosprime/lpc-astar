#ifndef _Astar_Included
#define _Astar_Included

// A* Path Data Structure
//
// Tracks a path as modeled by the A* algorithm.
//
// Usage: Variables containing A* path data structures can be found within
// pathfind data structures, as return values from immediately-successful
// pathfinding attempt, and as arguments to notification callbacks.  You
// would access the various data stored in it as path[Astar_Path_Nodes],
// path[Astar_Path_Edges], and so on.

// The list of nodes making up the path.
#define Astar_Path_Nodes                        0
// The edges between nodes in the path.  Each edge in the list represents
// the change necessary to move from the node in the corresponding position
// in the list of nodes to the next node.
#define Astar_Path_Edges                        1
// The path's distance from its target.
#define Astar_Path_Distance                     2
// The accumulated cost of the path.
#define Astar_Path_Cost                         3

#define Astar_Path_Fields                       4

// A* Cache Data Structure
//
// Tracks a path cache entry.

// The path itself (an Astar_Path_* structure).
#define Astar_Cache_Path                        0
// The number of times the cache entry has been requested.
#define Astar_Cache_Hits                        1
// The timestamp of the most recent time the cache entry was requested.
#define Astar_Cache_Timestamp                   2

#define Astar_Cache_Fields                      3

// A* Pathfind Data Structure
//
// Tracks the information defining a pathfinding attempt.
//
// Usage: Most of the behavioral control rules used by the module receive a
// pathfind data structure as their argument.  You would access its data as
// pathfind[Astar_Pathfind_Active_Node], pathfind[Astar_Pathfind_To], and
// so on.

// The starting node of the pathfinding attempt
#define Astar_Pathfind_From                     0
// The target node of the pathfinding attempt
#define Astar_Pathfind_To                       1
// The 'validate' argument astar_find_path() was called with, if any
#define Astar_Pathfind_Validate                 2
// The 'callback' argument astar_find_path() was called with, if any
#define Astar_Pathfind_Callback                 3
// The 'extra' argument astar_find_path() was called with, if any
#define Astar_Pathfind_Extra                    4
// A mapping of the nodes visited
#define Astar_Pathfind_Visited                  5
// The utime() when the pathfinding attempt started
#define Astar_Pathfind_Start_Time               6
// An array of the working path list for the pathfinding attempt
#define Astar_Pathfind_Paths                    7
// Set to the current path being worked with
#define Astar_Pathfind_Active_Path              8
// The utime() when the current pathfinding cycle, initial or call_out(), began
#define Astar_Pathfind_Cycle_Start              9
// The number of times the pathfinding process has run (each call_out() increments)
#define Astar_Pathfind_Cycle_Index              10
// The number of times the current pathfinding cycle has looped (checks run limit each time)
#define Astar_Pathfind_Cycle_Iterations         11
// Set to the current node being worked with
#define Astar_Pathfind_Active_Node              12
// Set to the current edge being worked with
#define Astar_Pathfind_Active_Edge              13
// Set to the final result of the pathfind (zero, a path structure, or a result code)
#define Astar_Pathfind_Result                   14
// Bitmask field that can contain Astar_Pathfind_Control_Flags as described below
#define Astar_Pathfind_Control_Flags            15

#define Astar_Pathfind_Fields                   16

// A* Pathfinder Control Flags
//
// Flag values for the Astar_Pathfind_Control_Flags field

// Signals the pathfind to terminate on its next processing cycle.  If this is the initial processing cycle, find_path() will
// return Astar_Result_Terminated.  No information about the pathfind will be cached.  Because this flag is only checked at
// certain points, it is still possible for a pathfind to finish or to fail for another reason after it has been set.
#define Astar_Pathfind_Control_Flag_Terminate   0x00000001
// If present, Astar_Pathfind_Callback will not be called at the end of processing
#define Astar_Pathfind_Control_Flag_Silent      0x00000002
// If present, any caching or cache lookups that would normally be done for the pathfind will be suppressed
#define Astar_Pathfind_Control_Flag_Uncache     0x00000004
// If present, the presence of a callback will not cause processing to continue via call_out()
#define Astar_Pathfind_Control_Flag_No_Continue 0x00000008

// A* Result Codes

// Result of astar_find_path() if pathfinding was moved to call_out(), either by the run limit being reached or by the neighbors 
// rule needing ongoing processing (returning Astar_Result_Processing), with a callback available
#define Astar_Result_Processing                 1
// Result of astar_find_path() if pathfinding was found to be impossible
#define Astar_Result_Impossible                 2
// Result of astar_find_path() if the run limit was encountered and no callback was available
#define Astar_Result_Cut_Off                    3
// Result of astar_find_path() if the neighbors rule needed ongoing processing and no callback was available
#define Astar_Result_Cannot_Continue            4
// Result of astar_find_path() if the pathfind terminated because of Astar_Pathfind_Control_Flag_Terminate
#define Astar_Result_Terminated                 5

// Tuning values

// Default value for the cache pruning threshold used by astar_prune_cache()
#define Astar_Prune_Cache_Default_Threshold     7200
// Each cache hit extends a cache entry's lifespan by this many seconds
#define Astar_Prune_Cache_Hit_Factor            60

#endif
