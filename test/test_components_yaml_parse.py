# Copyright 2024 Husarion sp. z o.o.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.


import xml.dom
import xml.dom.minidom
import os
import xacro
import xml
import yaml
from ament_index_python.packages import get_package_share_directory

ros_components_description = get_package_share_directory("ros_components_description")
xacro_path = os.path.join(ros_components_description, "test/component.urdf.xacro")


class ComponentsYamlParseUtils:
    __test__ = False

    def __init__(self, components_config_path: str) -> None:
        self.components_config_path = components_config_path

    def save_yaml(self, node: yaml.Node) -> None:
        with open(self.components_config_path, mode="w", encoding="utf-8") as file:
            yaml.dump(node, file, default_flow_style=False)

    def create_component(
        self,
        type: str,
        tf_prefix: str,
        namespace: str,
        name="None",
        parent_link="cover_link",
        xyz="0.0 0.0 0.0",
        rpy="0.0 0.0 0.0",
    ) -> dict:
        return {
            "type": type,
            "parent_link": parent_link,
            "xyz": xyz,
            "rpy": rpy,
            "tf_prefix": tf_prefix,
            "namespace": namespace,
            "name": name,
        }

    def try_to_parse_wrong_and_test(self, log: str) -> None:
        try:
            xacro.process_file(
                xacro_path, mappings={"components_config_path": self.components_config_path}
            )
            assert False, log
        except xacro.XacroException as e:
            f"{log} Error:\n{e}"
            assert True

    def try_to_parse_well_and_test(self, log: str) -> xml.dom.minidom.Document:
        doc = xml.dom.minidom.Document()
        try:
            doc = xacro.process_file(
                xacro_path, mappings={"components_config_path": self.components_config_path}
            )
            assert True
        except xacro.XacroException as e:
            assert False, f"{log}\nError:\n{e}"

        return doc

    def look_for_link_well_and_test(
        self, doc: xml.dom.minidom.Document, link_name: str, log: str
    ) -> None:
        links = doc.getElementsByTagName('link')
        for link in links:
            if link.getAttribute('name') == link_name:
                assert True
                return
        assert False, f"{log}"

    def look_for_link_wrong_and_test(
        self, doc: xml.dom.minidom.Document, link_name: str, log: str
    ) -> None:
        links = doc.getElementsByTagName('link')
        for link in links:
            if link.getAttribute('name') == link_name:
                assert False, f"{log}"
        assert True

    def look_for_sensor_name_well(
        self, doc: xml.dom.minidom.Document, link_name: str, sensor_name: str, log: str
    ) -> None:
        gazebos_tags = doc.getElementsByTagName('gazebo')
        for tag in gazebos_tags:
            if tag.getAttribute('reference') == link_name:
                sensors = doc.getElementsByTagName('sensor')
                for sensor in sensors:
                    if sensor.getAttribute('name') == sensor_name:
                        assert True
                        return

        assert False, f"{log}"

    def test_lidar(self, doc: xml.dom.minidom.Document, tf_prefix: str, namespace: str) -> None:
        link_name = "laser"
        if tf_prefix != "None":
            link_name = tf_prefix + "_" + link_name

        sensor_name = "rplidar_s1_sensor"
        if tf_prefix != "None":
            sensor_name = tf_prefix + "_" + sensor_name
        if namespace != "None":
            sensor_name = namespace + "/" + sensor_name

        self.look_for_link_well_and_test(doc, link_name, f"Could not find link: {link_name}")
        self.look_for_sensor_name_well(
            doc,
            link_name,
            sensor_name,
            f"Could not find sensor: {sensor_name}, referencing to link: {link_name} with config {self.components_config_path}",
        )

    def test_camera(
        self, doc: xml.dom.minidom.Document, tf_prefix: str, namespace: str, name: str
    ) -> None:
        link_name = name + "_orbbec_astra_link"
        if tf_prefix != "None":
            link_name = tf_prefix + "_" + link_name

        sensor_name = name + "_orbbec_astra_color"
        if tf_prefix != "None":
            sensor_name = tf_prefix + "_" + sensor_name
        if namespace != "None":
            sensor_name = namespace + "/" + sensor_name

        self.look_for_link_well_and_test(
            doc,
            link_name,
            f"Could not find link: {link_name} with config {self.components_config_path}",
        )

        self.look_for_sensor_name_well(
            doc,
            link_name,
            sensor_name,
            f"Could not find sensor: {sensor_name}, referencing to link: {link_name} with config {self.components_config_path}",
        )


def test_wrong_config(tmpdir_factory):
    dir = tmpdir_factory.mktemp("panther_bringup_test")
    components_config_path = dir.join("test_wrong_config.yaml")

    node = yaml.safe_load("a: 5")
    utils = ComponentsYamlParseUtils(components_config_path)

    utils.save_yaml(node)
    utils.try_to_parse_wrong_and_test(f"Xacro should not parse well for this yaml file: {node}")


def test_good_lidars_with_tf_prefix_and_namespace(tmpdir_factory):
    dir = tmpdir_factory.mktemp("panther_bringup_test")
    components_config_path = dir.join("test_good_lidars_with_tf_prefix_and_namespace.yaml")

    tf_prefixes = ["main_lidar", "additional_lidar"]
    namespaces = ["/main_lidar", "/additional_lidar"]

    utils = ComponentsYamlParseUtils(str(components_config_path))

    node = {
        "components": [],
    }
    for i in range(len(tf_prefixes)):
        node["components"].append(utils.create_component("LDR01", tf_prefixes[i], namespaces[i]))

    utils.save_yaml(node)

    doc = utils.try_to_parse_well_and_test(
        f"Could not parse without namespace:\n\t{node}\nIn file {components_config_path} with config {components_config_path}"
    )

    for i in range(len(tf_prefixes)):
        utils.test_lidar(doc, tf_prefixes[i], namespaces[i])


def test_good_cameras_with_tf_prefix_and_namespace(tmpdir_factory):
    dir = tmpdir_factory.mktemp("panther_bringup_test")
    components_config_path = dir.join("test_good_cameras_with_tf_prefix_and_namespace.yaml")

    tf_prefixes = ["main", "additional"]
    namespaces = ["/main", "/additional"]
    names = ["camera", "camera"]

    utils = ComponentsYamlParseUtils(str(components_config_path))

    node = {
        "components": [],
    }
    for i in range(len(tf_prefixes)):
        node["components"].append(
            utils.create_component(
                "CAM01",
                tf_prefixes[i],
                namespaces[i],
                names[i],
            )
        )

    utils.save_yaml(node)

    doc = utils.try_to_parse_well_and_test(
        f"Could not parse without namespace:\n\t{node}\nIn file {components_config_path}"
    )

    for i in range(len(tf_prefixes)):
        utils.test_camera(doc, tf_prefixes[i], namespaces[i], names[i])


def test_good_cameras_and_lidars_without_namespace_and_tf_prefix(tmpdir_factory):
    dir = tmpdir_factory.mktemp("panther_bringup_test")
    components_config_path = dir.join(
        "test_good_cameras_and_lidars_without_namespace_and_tf_prefix.yaml"
    )

    utils = ComponentsYamlParseUtils(str(components_config_path))

    names = ["camera_front", "camera_back"]
    node = {
        "components": [],
    }
    for i in range(len(names)):
        node["components"].append(utils.create_component("CAM01", "None", "None", names[i]))

    node["components"].append(utils.create_component("LDR01", "None", "None"))

    utils.save_yaml(node)

    doc = utils.try_to_parse_well_and_test(
        f"Could not parse without namespace:\n\t{node}\nIn file {components_config_path}"
    )

    for i in range(len(names)):
        utils.test_camera(doc, "None", "None", names[i])

    utils.test_lidar(doc, "None", "None")


def test_good_cameras_and_lidars_without_tf_prefix(tmpdir_factory):
    dir = tmpdir_factory.mktemp("panther_bringup_test")
    components_config_path = dir.join("test_good_cameras_and_lidars_without_tf_prefix.yaml")

    utils = ComponentsYamlParseUtils(str(components_config_path))

    names = ["camera_front", "camera_back"]
    node = {
        "components": [],
    }
    namespaces = ["/front", "/left"]
    lidar_namespace = "/main_lidar"

    for i in range(len(names)):
        node["components"].append(utils.create_component("CAM01", "None", namespaces[i], names[i]))

    node["components"].append(utils.create_component("LDR01", "None", lidar_namespace))

    utils.save_yaml(node)

    doc = utils.try_to_parse_well_and_test(
        f"Could not parse without namespace:\n\t{node}\nIn file {components_config_path}"
    )

    for i in range(len(names)):
        utils.test_camera(doc, "None", namespaces[i], names[i])

    utils.test_lidar(doc, "None", lidar_namespace)


def test_good_cameras_and_lidars_without_namespace(tmpdir_factory):
    dir = tmpdir_factory.mktemp("panther_bringup_test")
    components_config_path = dir.join("test_good_cameras_and_lidars_without_namespace.yaml")

    utils = ComponentsYamlParseUtils(str(components_config_path))

    names = ["camera_front", "camera_back"]
    node = {
        "components": [],
    }
    tf_prefixes = ["front", "left"]
    lidar_tf_prefix = "main_lidar"

    for i in range(len(names)):
        node["components"].append(
            utils.create_component(
                "CAM01",
                tf_prefixes[i],
                "None",
                names[i],
            )
        )

    node["components"].append(utils.create_component("LDR01", lidar_tf_prefix, "None"))

    utils.save_yaml(node)

    doc = utils.try_to_parse_well_and_test(
        f"Could not parse without namespace:\n\t{node}\nIn file {components_config_path}"
    )

    for i in range(len(names)):
        utils.test_camera(doc, tf_prefixes[i], "None", names[i])

    utils.test_lidar(doc, lidar_tf_prefix, "None")


def test_good_without_any_component(tmpdir_factory):
    dir = tmpdir_factory.mktemp("panther_bringup_test")
    components_config_path = dir.join("test_wrong_unfilled_params.yaml")

    utils = ComponentsYamlParseUtils(str(components_config_path))
    node = {
        "components": [],
    }

    utils.save_yaml(node)
    utils.try_to_parse_well_and_test(
        f"This configuration should be parsed :\n\t{node}\nIn file {components_config_path}"
    )

def test_wrong_empty_file(tmpdir_factory):
    dir = tmpdir_factory.mktemp("panther_bringup_test")
    components_config_path = dir.join("test_wrong_unfilled_params.yaml")

    utils = ComponentsYamlParseUtils(str(components_config_path))
    open(components_config_path, mode="w")

    utils.try_to_parse_wrong_and_test(
        f"This configuration should not be parsed.\nIn file {components_config_path}"
    )
