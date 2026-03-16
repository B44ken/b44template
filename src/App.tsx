import { App, Btn, Card, Col, Muted } from "b44ui"

export default function MyApp() {
  return <App>
    <Card>
      <Col>
        <h1>b44template</h1>
        <Muted>minimal vite + react + tailwind + b44ui</Muted>
        <Btn>button</Btn>
      </Col>
    </Card>
  </App>
}
