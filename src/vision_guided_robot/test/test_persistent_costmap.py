from vision_guided_robot.persistent_costmap import PersistentCostmap


def test_costmap_keeps_observed_cells_until_memory_timeout():
    costmap = PersistentCostmap(memory_time_s=5.0)

    costmap.update({(1, 2), (2, 2)}, now_s=10.0)

    assert costmap.active_cells(now_s=12.0) == {(1, 2), (2, 2)}
    assert costmap.active_cells(now_s=16.0) == set()


def test_costmap_refreshes_cell_timestamp_when_reobserved():
    costmap = PersistentCostmap(memory_time_s=5.0)

    costmap.update({(1, 2)}, now_s=10.0)
    costmap.update({(1, 2)}, now_s=14.0)

    assert costmap.active_cells(now_s=18.0) == {(1, 2)}
    assert costmap.active_cells(now_s=20.0) == set()


def test_costmap_signature_is_sorted_and_pruned():
    costmap = PersistentCostmap(memory_time_s=5.0)

    costmap.update({(4, 1), (1, 2)}, now_s=10.0)

    assert costmap.signature(now_s=11.0) == ((1, 2), (4, 1))
    assert costmap.signature(now_s=20.0) == tuple()
