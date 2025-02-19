import React, { Component } from 'react'
import Connection from './Connection';
import Teleoperation from './Teleoperation';
import { Row, Col, Container } from "react-bootstrap"
import RobotState from './RobotState';
import Map from './Map';
import Navigation from './Navigation';
import Camera from './Camera';

class Home extends Component {
    state = {};
    render() { 
        return (
            <div>
                <Container>
                    <h1 className='text-center mt-3'>Robot Control Page</h1>
                    <Row>
                        <Col>
                            <Connection />
                        </Col>
                    </Row>
                    <Row>
                        <h1 className='mt-4'></h1>
                    </Row>
                    <Row>
                        <Col>
                            <Teleoperation />
                            <RobotState />
                            <Navigation></Navigation>
                        </Col>
                        <Col>
                            <h1>MAP</h1>
                            <Map></Map>
                            {/* {" "}
                            <h1 className='mt-4'>CAMERA</h1>
                            <Camera></Camera> */}
                        </Col>
                    </Row>
                </Container>
            </div>
        );
    }
}
 
export default Home;