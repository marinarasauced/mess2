# DirectedGraph.msg
# this contains a directed graph consisting of an array of vertices and an array of edges

Vertex[] vertices           # vertices in the graph
Edge[] edges                # edges in the graph (indices correspond to the index of the vertex in vertices)
string type                 # directed or undirected