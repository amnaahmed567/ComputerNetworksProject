import networkx as nx
import matplotlib.pyplot as plt
import math
import numpy as np
from networkx import all_simple_edge_paths
from networkx import all_simple_paths
import pylab
import random

G=nx.Graph()

# adding nodes
node_list=[]
for i in range(22):
    G.add_node(i,pos=(i,0))
for i in range(22,50):
    node_list.append(i)

G.add_nodes_from(node_list)

# creating edges for connecting 20 nodes
for i in range(1,22):
    rand=np.random.randint(50,250)
    G.add_edge(i,i-1,weight=rand,cost=math.ceil(500/rand))

# creating list of remaining interfaces
starting_node=[]
for i in range(50):
    for j in range(5):
        if((i==0 and j==2) or (i==21 and j==2) or (i<21 and i>0 and j==3) ):
            break
        starting_node.append(i)

# removing self loops
np.random.shuffle(starting_node)
for i in range(0,len(starting_node),2):
    if(starting_node[i]==starting_node[i+1]):
        temp=starting_node[i+1]
        starting_node[i+1]=starting_node[i+2]
        starting_node[i+2]=temp

# completing remaining interface
for i in range(0,len(starting_node),2):
    rand=np.random.randint(50,250)
    G.add_edge(starting_node[i],starting_node[i+1],weight=0,cost=0)

# storing positions of all nodes
dic={}
for i in range(50):
    if(i>21):
        dic[i]=(np.random.randint(0,700),np.random.randint(-15,15))
    else:
        dic[i]=(i*10,0)

# arrays for storing data of 20 ietrations
hop_array=[]
cost_array=[]
overhead_array=[]
payload_array=[]
total_packets = 5   # 5 packets per iteration for 20 iterations

# 20 iterations
for j in range(20):
    random.seed()
    packets_dropped = 0
    # assigning random bandwidths to all edges
    for i in range(0,len(starting_node),2):
        rand_val=np.random.randint(50,250)
        u = starting_node[i]
        v = starting_node[i+1]

        if G.has_edge(u, v):
            G[u][v]['weight'] = rand_val
            G[u][v]['cost'] = math.ceil(500 / rand_val)
            
    for i in range(1,22):
        rand=np.random.randint(50,250)
        G[i][i-1]['weight']=rand
        G[i][i-1]['cost']=math.ceil(500/rand)
    # Simulating link breakage for packet drop
    edges_to_break = list(G.edges())
    random_edge = random.choice(edges_to_break)
    if np.random.rand() < 0.2:  # Example: Break a link with 10% probability
        # Choose a random edge to break
        
        if edges_to_break:
            
            G.remove_edge(*random_edge)
            packets_dropped += 1
    # Remove edge to restore the graph for the next iteration
    if edges_to_break:
       G.add_edge(*random_edge)  # Restore the broken edge for the next iteration
    # configuring rendered image
    pylab.figure(1,figsize=(70,30))

    # plotting graph
    nx.draw(G,pos=dic,with_labels=True,node_color='#34d8eb',node_size=400,font_size=8)
    labels = nx.get_edge_attributes(G,'weight')
    nx.draw_networkx_edge_labels(G,pos=dic,edge_labels=labels,font_size=12)

    # finding shortest path
    path = nx.shortest_path(G, source=0, target=21, weight='cost', method='dijkstra')
    path_cost = nx.shortest_path_length(G, source=0, target=21, weight='cost', method='dijkstra')
    # Function to break edges
    
    edges = [(path[i], path[i + 1]) for i in range(len(path) - 1)]
    num_edges_to_break = random.randint(0, min(4, len(edges)))  # Randomly choose up to 4 edges to break

    edges_to_break = random.sample(edges, num_edges_to_break)  # Randomly select edges to break

    for edge in edges_to_break:
        G.remove_edge(*edge)  # Break the edge
        # Simulate packet drop due to the broken link (you might adjust this logic if needed)
        if np.random.rand() < 0.2:
            packets_dropped += 1  # Increment packet drop count
    # Remove edge to restore the graph for the next iteration
    for edge in edges_to_break:
       G.add_edge(*edge)  # Restore the broken edge for the next iteration
    # finding edges involved in the shortest path
    edge=[]
    prev=0
    for i in path:
        if(i!=0):
            edge.append((prev,i))
        prev=i

    # finding hops, cost, overhead of the network
    packet_size=1000
    hello_packet_size=50
    total_payload=0
    total_cost=0

    # sending 5 packets
    for i in range(5):
        total_cost+=path_cost
        total_payload+=packet_size
    hop_array.append(len(edge))
    cost_array.append(total_cost/5)
    overhead_array.append(hello_packet_size*len(edge))
    payload_array.append(total_payload/5)
    
    # Calculate total packets sent over all iterations
    
# Calculate Packet Delivery Ratio (PDR)
    pdr = (total_packets - packets_dropped) / total_packets

    # giving the required answers
    print("Packets Dropped:",packets_dropped)
    print("The optimal path is:")
    print(path)
    print("The average cost is: "+str(total_cost/5))
    print("Total number of hops are: "+str(len(edge)))
    print("Total overhead of the network is: "+str(len(edge))+" hello packets.")
    print("That is for each connection established bw source and dest. we have to transmit "+str(len(edge))+" hello packets of 50 bytes in our case.")
    print("Packet Delivery Ratio (PDR):", pdr)
    # customizing the graph
    nx.draw_networkx_nodes(G,pos=dic,nodelist=path,node_size=800,node_color='red')
    nx.draw_networkx_nodes(G,pos=dic,nodelist=[0,21],node_size=800,node_color='#4dff7c')
    nx.draw_networkx_edges(G,pos=dic,edgelist=edge,width=5,edge_color='yellow',arrows=True)
    nx.draw_networkx_edge_labels(G,pos=dic,edge_labels=labels,font_size=12)
    plt.savefig("figure"+str(j)+".png")
    plt.close()
    #plt.show()
    

# printing average hops, cost and overhead of the system
print("Dijkstra algorithm was used to find the optimal path and there were 20 iterations for 50 nodes.")
print("The average hop count was: "+str(sum(hop_array)/20))
print("The average cost was: "+str(sum(cost_array)/20))
print("The average hello packets carried were: "+str(sum(overhead_array)/(20*50)))
print("The average overhead(hello packets) bytes carried were: "+str(sum(overhead_array)/20))
print("The average payload bytes carried were: "+str(sum(payload_array)/20))

input("")
