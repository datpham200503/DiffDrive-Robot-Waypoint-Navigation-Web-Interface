import React, { Component } from 'react'
import Config from '../scripts/config';

class Camera extends Component {
    state = { ros: null } 

    constructor() {
        super();
        this.state.ros = new window.ROSLIB.Ros();
        this.view_map = this.set_camera.bind(this);
    }

    init_connection() {
        this.state.ros.on("connection", () => {
            console.log("connection established in Camera!");
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
        this.set_camera();
    }

    set_camera() {
        var cameraViewer = new window.MJPEGCANVAS.Viewer({
            ros: this.state.ros,
            divID: 'mjpeg',
            host: Config.ROSBRIDGE_SERVER_IP,
            width: 640,
            height: 480,
            topic: '/image_raw',
            port: '8080',
        });
    }

    render() { 
        return (
            <div id="mjpeg">Viewer</div>
        );
    }
}
 
export default Camera;