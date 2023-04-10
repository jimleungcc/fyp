class TreeNode:
    def __init__(self, value):
        self.value = value
        self.children = []

    def add_child(self, child):
        self.children.append(child)

    def is_leaf(self):
        return len(self.children) == 0

    def add_children_to_deepest_leaves(self, values):
        """
        Adds children to all deepest leaf nodes in the tree.

        Args:
        - values (List): A list of values for the new children nodes.

        Returns:
        - None
        """
        max_depth = 0
        deepest_leaves = []

        def dfs(node, depth):
            nonlocal max_depth, deepest_leaves

            if node.is_leaf():
                if depth > max_depth:
                    max_depth = depth
                    deepest_leaves = [node]
                elif depth == max_depth:
                    deepest_leaves.append(node)
            else:
                for node_child in node.children:
                    dfs(node_child, depth + 1)

        dfs(self, 0)

        for leaf in deepest_leaves:
            for value in values:
                child = TreeNode(value)
                leaf.add_child(child)

    def add_child_to_all_nodes(self, node_value, child_value):
        target_nodes = []

        def find_nodes_with_value(tree_node, value):
            if tree_node.value == value:
                target_nodes.append(tree_node)
            for child in tree_node.children:
                find_nodes_with_value(child, value)

        find_nodes_with_value(self, node_value)

        if not target_nodes:
            print(f"No TreeNode found with value {node_value}")
        else:
            for node in target_nodes:
                child_node = TreeNode(child_value)
                node.add_child(child_node)

    def get_paths_to_leaves(self):
        """
        Returns a list of arrays, each containing the values in the path from the root to a leaf.

        Returns:
        - list of arrays: Each array contains the values in the path from the root to a leaf.
        """

        def dfs(node, tree_path):
            if node.is_leaf():
                # Leaf node, add the path to the result list
                return [tree_path + [node.value]]
            else:
                # Non-leaf node, continue recursively
                result = []
                for child in node.children:
                    result.extend(dfs(child, tree_path + [node.value]))
                return result

        return dfs(self, [])