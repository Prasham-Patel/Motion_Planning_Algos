# RBE 550 Motion Planning

# Search Algorithms Implementation

## PRM

Four sampling method have been applied, such that the produce the given number of points. The new node makes connection with nearest 8 nodes given that the path formed is not obstructed. They are as listed below:

### Uniform Sampling:

![image](https://user-images.githubusercontent.com/91484142/165418151-18f1fcbe-2fcb-4622-8c84-423560dfe55b.png)

**Advantage:** 
- 	Simple method which can cover most of the part of the map 				consistently, as there is no randomness in sampling.
	
**Disadvantage:**
		-	Might miss to sample narrow path, as specially when the path is 					between two sampling lengths. 
		-	Need small sampling length for more complex environment.


### Random Sampling:
![image](https://user-images.githubusercontent.com/91484142/165418161-d0d0aad5-386e-45bd-aa03-9b3a7b17bd44.png)

 

**Advantage:** 
- 	Simplest method which can cover most of the part of the map .
	
**Disadvantage:**
		-	Might miss to sample narrow path.
		-	More number of samples required for complex environment.

### Gaussian Sampling:

![image](https://user-images.githubusercontent.com/91484142/165418183-63e3ac90-0497-44cb-b580-201b1f9dc407.png)
 

**Advantage:** 
- 	Gives an idea about the location of obstacles.
-	Can find path with narrow passages.
	
**Disadvantage:**
		-	Generated path flows around obstacles, like bug algorithm. 

### Bridge sampling:

![image](https://user-images.githubusercontent.com/91484142/165418382-c06c1db0-d3cd-4151-b3b8-89b86bd6d4fa.png)


 
**Advantage:**
-	Can find path with narrow passages.
	
**Disadvantage:**
		-	Only samples near narrow passages, bad for more open 					environment

### RRT

![image](https://user-images.githubusercontent.com/91484142/165418226-299f4d79-e445-495c-9be0-b4dbf7e857de.png)
RRT algorithm randomly generates nodes from start position and connects them to the nearest available node given path is not obstructed. If the path length is more than step-size specified than a new node is created on the previous path such that its distance from parent node is equal to step size. Below is the image of implementation:

 
### RRT*

![image](https://user-images.githubusercontent.com/91484142/165418251-657d4a4c-f37c-40f9-8e55-92f1d36b3aa1.png)

RRT* has 2 extra features than RRT, those are:

-	Connecting the new node with the node which results into least cost rather than connecting to the nearest one.
-	Rewiring the nodes within a given radius of the new node to the parent node of new node if the over all cost decreases. 	

This changes result into a over all more optimum path from start to goal.


# Advance Algorithm Implementation

## D* Algorithm

The D* Algorithm or Dynamic A* is implemented when re-planning of the path is required frequently. The D* does it by updating the ‘h’ value of a node whenever it discovered as an obstacle to ‘inf’. Thus, the cost of moving that node to its neighboring node increase to ‘inf’. All the nodes surrounding the new obstacle node are again put into open list.

It should be noted that the priority queue is ordered according to ‘k’ value of the node. The ‘k’ value is the minimum ‘h’ value of the node so far.

D* works faster than A* because it does not do complete replanning, rather it updates the path around the newly discovered obstacle node using the previously calculated ‘k’ value and ‘h’ value for other nodes.



| ![image](https://user-images.githubusercontent.com/91484142/163594451-668a1cb5-fac3-4687-92d3-4d4dd6610f01.png)|
|:--:|
| <b>**Figure 1: final path for map-1**</b>|

| ![image](https://user-images.githubusercontent.com/91484142/163594464-ee4a2222-1372-41dd-9749-a24ae1896d90.png)|
|:--:|
|<b>**Figure 2: final path for map-1**</b>|


| ![image](https://user-images.githubusercontent.com/91484142/163594485-a51f027d-290f-4d34-8b65-d51583150225.png)|
|:--:|
|<b>**Figure 3: final path for map-3**</b>| 

## Informed RRT*

RRT* when generating new samples randomly even when it has a known path, while Informed RRT* generates new sample randomly within an ellipse whose dimensions are decided on the basis of existing path.

 

| ![image](https://user-images.githubusercontent.com/91484142/163594896-8d542165-c2e6-4341-9e08-ea24aca36772.png)|
|:--:|
|<b>**Figure 4: Informed RRT***</b>| 

| ![image](https://user-images.githubusercontent.com/91484142/163594913-5e29bc7c-7eac-4e20-9706-1ee64d523e74.png)|
|:--:|
|<b>**Figure 5: RRT***</b>| 

| ![image](https://user-images.githubusercontent.com/91484142/163594934-a3c17c09-3657-4ecf-a3c2-93f7ae9afbc8.png)|
|:--:|
|<b>**Figure 6: RRT**</b>| 


As can be seen from the figure the node exploration in Informed RRT* is much more focused around the discovered path. While for RRT* the sampling is almost evenly distributed on the entire map
