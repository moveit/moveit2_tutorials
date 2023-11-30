.. _Parallel Containers:

###################
Parallel Containers
###################

Parallel containers combine a set of stages to allow planning alternate solutions.

Three stages provided by MTC to use within a parallel container

* Alternatives - Solution of all children are collected and sorted by cost.

* Fallback - A fallback container executes children stages in order until one of them returns success or all stages return failure

* Merger - Solutions of all children (actuating disjoint groups) are planned and executed parallelly.

Alternatives
^^^^^^^^^^^^


Fallback
^^^^^^^^


Merger
^^^^^^
