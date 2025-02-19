import React, { Component } from 'react';
import { Button, ButtonGroup, Container } from 'react-bootstrap';
import Config from '../scripts/config'

class Navigation extends Component {
    state = {
        sequence: [],
        disableButtons: {},
        run: false,
    };

    constructor(props) {
        super(props);
        this.state.ros = new window.ROSLIB.Ros();
        this.init_connection();
    };

    init_connection() {
        this.state.ros.on("connection", () => {
            console.log("connection established in Navigation!");
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

    handleButtonClick = (number) => {
        this.setState((prevState) => ({
            sequence: [...prevState.sequence, number],
            disableButtons: { ...prevState.disableButtons, [number]: true },
        }));
    };

    handleClear = () => {
        this.setState({ sequence: [], disableButtons: {} , run: false }, () => {
            console.log('run:', this.state.run); 
            console.log('path: ', this.state.sequence);
        });
    };

    handleStart = () => {
        this.setState({ run: true }, () => {
            var path = new window.ROSLIB.Topic({
                ros: this.state.ros,
                name: "/path",
                messageType: "std_msgs/Int32MultiArray"
            });
    
            var array = new window.ROSLIB.Message({
                data: this.state.sequence,
            });
    
            path.publish(array);
        });
    };

    render() {
        const { sequence, disableButtons } = this.state;
        return (
            <Container className='mt-4'>
                <h1 className='mb-4'>Navigation Controller</h1>
                <ButtonGroup className='d-flex justify-content-center mb-3'>
                    {[1, 2, 3, 4, 5, 6, 7, 8, 9].map((number) => (
                        <Button
                            key={number}
                            variant='primary'
                            className='mx-1'
                            disabled={disableButtons[number]}
                            onClick={() => this.handleButtonClick(number)}
                        >
                            {number}
                        </Button>
                    ))}
                </ButtonGroup>
                <div className='mb-3'>
                    <Button variant='danger' onClick={this.handleClear}>
                        Clear
                    </Button>
                    <Button 
                        variant='success' 
                        onClick={this.handleStart} 
                        className='mx-2'
                    >
                        Start
                    </Button>
                </div>
                <h4>PATH: {sequence.length > 0 ? sequence.join(' -> ') : 'None'}</h4>
            </Container>
        );
    }
}

export default Navigation;
