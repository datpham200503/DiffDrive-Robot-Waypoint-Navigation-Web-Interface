import React, { Component } from 'react'
import Config from '../scripts/config'

class Map extends Component {
    state = { ros: null } 

    constructor() {
        super();
        this.state.ros = new window.ROSLIB.Ros();
        this.view_map = this.view_map.bind(this);
    }

    init_connection() {
        this.state.ros.on("connection", () => {
            console.log("connection established in Teleoperation!");
            this.setState({ connected: true });
        });

        this.state.ros.on("close", () => {
            console.log("connection is closed!");
            this.setState({ connected: false });

            setTimeout(() => {
                try {
                    this.state.ros.connect(
                        "ws://" + Config.ROSBRIDGE_SERVER_IP + ":" + Config.ROSBRIDGE_SERVER_PORT
                    );
                } catch (error) {
                    console.log("connection problem", error);
                }
            }, Config.RECONNECTION_TIMER);
        });

        try {
            this.state.ros.connect(
                "ws://" + Config.ROSBRIDGE_SERVER_IP + ":" + Config.ROSBRIDGE_SERVER_PORT
            );
        } catch (error) {
            console.log("connection problem", error);
        }
    }

    componentDidMount() {
        this.init_connection();
        this.view_map();
    }

    view_map() {
        var viewer = new window.ROS2D.Viewer({
            divID: "nav_div",
            width: 650,
            height: 650,
        });

        var navClient = new window.NAV2D.OccupancyGridClientNav({
            ros: this.state.ros,
            rootObject: viewer.scene,
            viewer: viewer,
            serverName: "/move_base",
            withOrientation: true,
        });
    }

    render() { 
        return (
                <div id="nav_div">Viewer</div>
        );
    }
}
 
export default Map;