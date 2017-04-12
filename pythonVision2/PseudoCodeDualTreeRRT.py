


def DualTreeRRT(StartConfig, GoalConfig, WorkspaceTree, StatespaceTree):
	while(True):
		if GoalBias:
			Sample = GoalConfig
			NearestNeighbor = FindNearestNeighbor(NewNodeList)
			NewNodeList.clear()
		else
			Sample = SampleWorkspace()
			NearestNeighbor = FindNearestNeighbor(WorkspaceTree)

		for Node in [NearestNeighbor NearestNeighbor.Parents]:
			if ConnectWithDynamics(Node, Node -> State, Sample)
				if GoalBias:
					return WorkspaceTree, StatespaceTree
				break


