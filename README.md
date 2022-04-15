# RBE 550 Motion Planning
# Advance Algorithm Implementation

Submitted by: Prasham Patel

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
