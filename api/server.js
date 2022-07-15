const express = require('express')
const app = express()
const port = 3000

app.get('/', (req, res) => {
  res.send('Hello World!')
})
// POST method route
app.post('/', (req, res) => {
    console.log("lkfds")
    res.send('POST request to the homepage')
  })
app.listen(port, () => {
  console.log(`Example app listening on port ${port}`)
})