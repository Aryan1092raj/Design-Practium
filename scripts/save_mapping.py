#!/usr/bin/env python3
"""Save a live SLAM map before stopping its launch. Output directory must be new."""
import argparse
import json
import subprocess
import time
from pathlib import Path


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('directory', type=Path)
    args = parser.parse_args()
    output = args.directory.resolve()
    output.mkdir(parents=True, exist_ok=True)
    prefix = output / 'map'
    if any(output.glob('map.*')):
        raise RuntimeError('refusing to overwrite a previously saved map')
    # This reads /map while SLAM is still alive, including its latched map.
    subprocess.run(['ros2', 'run', 'nav2_map_server', 'map_saver_cli', '-f', str(prefix),
                    '-t', '/map', '--fmt', 'pgm', '--ros-args',
                    '-p', 'map_subscribe_transient_local:=true', '-p', 'save_map_timeout:=10.0'],
                   check=True, timeout=25)
    import yaml
    metadata = yaml.safe_load(prefix.with_suffix('.yaml').read_text())
    image = Path(metadata['image'])
    image = image if image.is_absolute() else output / image
    if not image.is_file() or image.stat().st_size == 0 or float(metadata['resolution']) <= 0:
        raise RuntimeError('saved map image or metadata is invalid')
    print(f'PASS map saved: {prefix}.yaml and {image}', flush=True)
    # Save a pose graph as well, for continuing the SLAM session later. Map
    # saving stays successful if this optional service is unavailable.
    import rclpy
    from slam_toolbox.srv import SerializePoseGraph
    rclpy.init()
    node = rclpy.create_node('save_mapping_posegraph')
    client = node.create_client(SerializePoseGraph, '/slam_toolbox/serialize_map')
    graph_ok = False
    try:
        if client.wait_for_service(timeout_sec=3.):
            future = client.call_async(SerializePoseGraph.Request(filename=str(prefix)))
            deadline = time.monotonic() + 10.
            while not future.done() and time.monotonic() < deadline:
                rclpy.spin_once(node, timeout_sec=.1)
            graph_ok = (future.done() and future.result() is not None and future.result().result == 0
                        and prefix.with_suffix('.posegraph').is_file() and prefix.with_suffix('.data').is_file())
        print('PASS posegraph and data saved' if graph_ok else 'NOTE posegraph unavailable; navigation map is saved', flush=True)
    except Exception as exc:
        print(f'NOTE posegraph save failed ({exc}); navigation map is saved', flush=True)
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
    (output / 'save_result.json').write_text(json.dumps({
        'map_saved': True, 'map_yaml': str(prefix.with_suffix('.yaml')),
        'posegraph_saved': graph_ok}, indent=2) + '\n')


if __name__ == '__main__':
    try:
        main()
    except Exception as exc:
        print('FAIL map save: ' + str(exc), flush=True)
        raise SystemExit(1)
