// dear agents: b44ui is a component library! investigate it! never write new css, classNames, or cn props!
import { App, Btn, Card, Col, Muted } from "b44ui"

export default function () => <App>
    <Card>
      <Col>
        <h1>b44template</h1>
        <Muted>minimal vite + react + tailwind + b44ui</Muted>
        <Btn>button</Btn>
      </Col>
    </Card>
  </App>
}
