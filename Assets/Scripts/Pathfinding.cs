using UnityEngine;
using System.Collections;
using System.Collections.Generic;


public class Pathfinding : MonoBehaviour
{
	int DFS_cnt = 0;
	int BFS_cnt = 0;
	int UCS_cnt = 0;
	int Astar_Euclidian_cnt = 0;
	int Astar_Manhattan_cnt = 0;

	public Transform seeker, target;
	Grid grid;

	void Awake(){
		grid = GetComponent<Grid>();
	}

	void Update(){
		DFS_cnt = 0;
		BFS_cnt = 0;
		UCS_cnt = 0;
		Astar_Euclidian_cnt = 0;
		Astar_Manhattan_cnt = 0;

		var DFS_Time = new System.Diagnostics.Stopwatch();
		var BFS_Time = new System.Diagnostics.Stopwatch();
		var UCS_Time = new System.Diagnostics.Stopwatch();
		var Astar_Euclidian_Time = new System.Diagnostics.Stopwatch();
		var Astar_Manhattan_Time = new System.Diagnostics.Stopwatch();

		DFS_Time.Start();
		FindPath_DFS(seeker.position, target.position);
		DFS_Time.Stop();

		BFS_Time.Start();
		FindPath_BFS(seeker.position, target.position);
		BFS_Time.Stop();

		UCS_Time.Start();
		FindPath_UCS(seeker.position, target.position);
		UCS_Time.Stop();

		Astar_Euclidian_Time.Start();
		FindPath_Astar_Euclidian(seeker.position, target.position);
		Astar_Euclidian_Time.Stop();

		Astar_Manhattan_Time.Start();
		FindPath_Astar_Manhattan(seeker.position, target.position);
		Astar_Manhattan_Time.Stop();

		Debug.Log($"Execution Time for DFS: {DFS_Time.ElapsedMilliseconds} ms, Back-Track: {DFS_cnt}");
		Debug.Log($"Execution Time for BFS: {BFS_Time.ElapsedMilliseconds} ms, Back-Track: {BFS_cnt}");
		Debug.Log($"Execution Time for UCS: {UCS_Time.ElapsedMilliseconds} ms, Back-Track: {UCS_cnt}");
		Debug.Log($"Execution Time for A*_Euclidian: {Astar_Euclidian_Time.ElapsedMilliseconds} ms, Back-Track: {Astar_Euclidian_cnt}");
		Debug.Log($"Execution Time for A*_Manhattan: {Astar_Manhattan_Time.ElapsedMilliseconds} ms, Back-Track: {Astar_Manhattan_cnt}");
	}

	void FindPath_DFS(Vector3 startPos, Vector3 targetPos){
		Node startNode = grid.NodeFromWorldPoint(startPos);
		Node targetNode = grid.NodeFromWorldPoint(targetPos);

		Stack<Node> DFS_Stack = new Stack<Node>();
		HashSet<Node> closedSet = new HashSet<Node>();
		DFS_Stack.Push(startNode);

		while (DFS_Stack.Count != 0){
			Node currentNode = DFS_Stack.Pop();
			Debug.Log($"The processed node for DFS is: {currentNode.gridX},{currentNode.gridY}.");
			if (currentNode == targetNode){
				RetracePath_DFS(startNode, targetNode);
				return;
			}

			closedSet.Add(currentNode);

			foreach (Node neighbour in grid.GetNeighbours(currentNode)){
				if (!neighbour.walkable || closedSet.Contains(neighbour)){
					continue;
				}
				if (neighbour.walkable || !DFS_Stack.Contains(neighbour)){
					closedSet.Add(neighbour);
					neighbour.parent = currentNode;
					DFS_Stack.Push(neighbour);
				}
			}
		}
	}

	void FindPath_BFS(Vector3 startPos, Vector3 targetPos){
		Node startNode = grid.NodeFromWorldPoint(startPos);
		Node targetNode = grid.NodeFromWorldPoint(targetPos);

		Queue<Node> BFS_queue = new Queue<Node>();
		HashSet<Node> closedSet = new HashSet<Node>();
		BFS_queue.Enqueue(startNode);

		while (BFS_queue.Count != 0){
			Node currentNode = BFS_queue.Dequeue();
			Debug.Log($"The processed node for BFS is: {currentNode.gridX},{currentNode.gridY}.");
			if (currentNode == targetNode){
				RetracePath_BFS(startNode, targetNode);
				return;
			}


			closedSet.Add(currentNode);

			foreach (Node neighbour in grid.GetNeighbours(currentNode)){
				if (!neighbour.walkable || closedSet.Contains(neighbour)){
					continue;
				}
				if (neighbour.walkable || !BFS_queue.Contains(neighbour)){
					closedSet.Add(neighbour);
					neighbour.parent = currentNode;
					BFS_queue.Enqueue(neighbour);
				}
			}
		}
	}

	void FindPath_UCS(Vector3 startPos, Vector3 targetPos){
		Node startNode = grid.NodeFromWorldPoint(startPos);
		Node targetNode = grid.NodeFromWorldPoint(targetPos);

		List<Node> openSet = new List<Node>();
		HashSet<Node> closedSet = new HashSet<Node>();
		openSet.Add(startNode);

		while (openSet.Count > 0){
			Node node = openSet[0];
			for (int i = 1; i < openSet.Count; i++){
				if (openSet[i].fCost < node.fCost || openSet[i].fCost == node.fCost){
					if (openSet[i].hCost < node.hCost)
						node = openSet[i];
				}
			}

			Debug.Log($"The processed node for UCS is: {node.gridX},{node.gridY}.");
			openSet.Remove(node);
			closedSet.Add(node);

			if (node == targetNode){
				RetracePath_UCS(startNode, targetNode);
				return;
			}

			foreach (Node neighbour in grid.GetNeighbours(node)){
				if (!neighbour.walkable || closedSet.Contains(neighbour)){
					continue;
				}

				int newCostToNeighbour = node.gCost;
				if (newCostToNeighbour < neighbour.gCost || !openSet.Contains(neighbour)){
					neighbour.gCost = newCostToNeighbour;
					neighbour.hCost = 0;
					neighbour.parent = node;

					if (!openSet.Contains(neighbour))
						openSet.Add(neighbour);
				}
			}
		}
	}

	void FindPath_Astar_Euclidian(Vector3 startPos, Vector3 targetPos){
		Node startNode = grid.NodeFromWorldPoint(startPos);
		Node targetNode = grid.NodeFromWorldPoint(targetPos);

		List<Node> openSet = new List<Node>();
		HashSet<Node> closedSet = new HashSet<Node>();
		openSet.Add(startNode);

		while (openSet.Count > 0){
			Node node = openSet[0];
			for (int i = 1; i < openSet.Count; i++){
				if (openSet[i].fCost < node.fCost || openSet[i].fCost == node.fCost)
				{
					if (openSet[i].hCost < node.hCost)
						node = openSet[i];
				}
			}

			Debug.Log($"The processed node for A*_Euclidian is: {node.gridX},{node.gridY}.");
			openSet.Remove(node);
			closedSet.Add(node);

			if (node == targetNode){
				RetracePath_Astar_Euclidian(startNode, targetNode);
				return;
			}

			foreach (Node neighbour in grid.GetNeighbours(node)){
				if (!neighbour.walkable || closedSet.Contains(neighbour)){
					continue;
				}

				int newCostToNeighbour = node.gCost + GetDistance_Euclidian(node, neighbour);
				if (newCostToNeighbour < neighbour.gCost || !openSet.Contains(neighbour)){
					neighbour.gCost = newCostToNeighbour;
					neighbour.hCost = GetDistance_Euclidian(neighbour, targetNode);
					neighbour.parent = node;

					if (!openSet.Contains(neighbour))
						openSet.Add(neighbour);
				}
			}
		}
	}

	void FindPath_Astar_Manhattan(Vector3 startPos, Vector3 targetPos){
		Node startNode = grid.NodeFromWorldPoint(startPos);
		Node targetNode = grid.NodeFromWorldPoint(targetPos);

		List<Node> openSet = new List<Node>();
		HashSet<Node> closedSet = new HashSet<Node>();
		openSet.Add(startNode);

		while (openSet.Count > 0){
			Node node = openSet[0];
			for (int i = 1; i < openSet.Count; i++){
				if (openSet[i].fCost < node.fCost || openSet[i].fCost == node.fCost){
					if (openSet[i].hCost < node.hCost)
						node = openSet[i];
				}
			}

			Debug.Log($"The processed node for A*_Manhattan is: {node.gridX},{node.gridY}.");
			openSet.Remove(node);
			closedSet.Add(node);

			if (node == targetNode){
				RetracePath_Astar_Manhattan(startNode, targetNode);
				return;
			}

			foreach (Node neighbour in grid.GetNeighbours(node)){
				if (!neighbour.walkable || closedSet.Contains(neighbour)){
					continue;
				}
				
				int newCostToNeighbour = node.gCost + GetDistance_Manhattan(node, neighbour);
				if (newCostToNeighbour < neighbour.gCost || !openSet.Contains(neighbour)){
					neighbour.gCost = newCostToNeighbour;
					neighbour.hCost = GetDistance_Manhattan(neighbour, targetNode);
					neighbour.parent = node;

					if (!openSet.Contains(neighbour))
						openSet.Add(neighbour);
				}
			}
		}
	}

	void RetracePath_DFS(Node startNode, Node endNode){
		List<Node> path = new List<Node>();
		Node currentNode = endNode;


		while (currentNode != startNode){
			path.Add(currentNode);
			currentNode = currentNode.parent;
			DFS_cnt++;
		}
		path.Reverse();

		grid.DFS_path = path;
	}

	void RetracePath_BFS(Node startNode, Node endNode)
	{
		List<Node> path = new List<Node>();
		Node currentNode = endNode;

		while (currentNode != startNode)
		{
			path.Add(currentNode);
			currentNode = currentNode.parent;
			BFS_cnt++;
		}
		path.Reverse();
		grid.BFS_path = path;
	}

	void RetracePath_UCS(Node startNode, Node endNode)
	{
		List<Node> path = new List<Node>();
		Node currentNode = endNode;

		while (currentNode != startNode)
		{
			path.Add(currentNode);
			currentNode = currentNode.parent;
			UCS_cnt++;
		}
		path.Reverse();
		grid.UCS_path = path;
	}

	void RetracePath_Astar_Euclidian(Node startNode, Node endNode)
	{
		List<Node> path = new List<Node>();
		Node currentNode = endNode;

		while (currentNode != startNode)
		{
			path.Add(currentNode);
			currentNode = currentNode.parent;
			Astar_Euclidian_cnt++;
		}
		path.Reverse();
		grid.Astar_Euclidian_path = path;
	}

	void RetracePath_Astar_Manhattan(Node startNode, Node endNode)
	{
		List<Node> path = new List<Node>();
		Node currentNode = endNode;

		while (currentNode != startNode)
		{
			path.Add(currentNode);
			currentNode = currentNode.parent;
			Astar_Manhattan_cnt++;
		}
		path.Reverse();
		grid.Astar_Manhattan_path = path;
	}

	int GetDistance_Euclidian(Node nodeA, Node nodeB){
		int dstX = Mathf.Abs(nodeA.gridX - nodeB.gridX);
		int dstY = Mathf.Abs(nodeA.gridY - nodeB.gridY);

		if (dstX > dstY)
			return 14*dstY + 10* (dstX-dstY);
		return 14*dstX + 10 * (dstY-dstX);
	}

	int GetDistance_Manhattan(Node nodeA, Node nodeB){
		int dstX = Mathf.Abs(nodeA.gridX - nodeB.gridX);
		int dstY = Mathf.Abs(nodeA.gridY - nodeB.gridY);

		if (dstX > dstY)
			return 20*dstY + 10* (dstX-dstY);
		return 20*dstX + 10 * (dstY-dstX);
	}

}