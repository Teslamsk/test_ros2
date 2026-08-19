package org.example.can.dictionary;

import lombok.Getter;

@Getter
public enum Node {
    ROTATE_LIDAR_Z(0x01, "Шаговый двигатель лидара вращения в оси Z");


    Node(int nodeId, String description) {
        this.nodeId = nodeId;
        this.description = description;
    }

    private final int nodeId;
    private final String description;
}
