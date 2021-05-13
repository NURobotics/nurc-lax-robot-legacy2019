var createError = require('http-errors');
var express = require('express');
var path = require('path');
var cookieParser = require('cookie-parser');
var logger = require('morgan');

var indexRouter = require('./routes/index');
var usersRouter = require('./routes/users');

var WebSocket = require('ws');

var app = express();

const ws = new WebSocket('ws://127.0.0.1:8080/');

ws.on('open', function open() {
  ws.send('something');
  console.log('sent message to server');
});

ws.on('message', function incoming(data) {
  console.log(data);
});

// view engine setup
app.set('views', path.join(__dirname, 'views'));
app.set('view engine', 'jade');

app.use(logger('dev'));
app.use(express.json());
app.use(express.urlencoded({ extended: false }));
app.use(cookieParser());
app.use(express.static(path.join(__dirname, 'public')));

app.use('/', indexRouter);
app.use('/users', usersRouter);

// Functions needed start here

app.get(/*url, function*/ {
  // press left button, move left
});

app.get( /*url, function*/ {
  // press right button, move right
});

app.get(/*url, function*/ {
  // press up button, move forward
});

app.get(/*url, function*/ {
  //press down button, move back
});

app.get(/*url, function*/{
  //press ++(Z) button, speed up
});

app.get(/*url, function*/ {
  //press HALT(X) button, hit the brakes
});

app.get(/*url, function*/{
  //press --(C) button, slow down
});

app.get(/*url, function*/{
  // press 1 (Begin Visuals), turn on camera
});

app.get(/*url, function*/{
  // press 2, start sending information about speed, and ball coordinates and if goalie managed to catch it
});

app.get(/*url, function*/{
  // press 3, stop sending information
});

app.get(/*url, function*/{
  // press 4 (End Visuals), turn off camera
});

//Functions needed ends here

// catch 404 and forward to error handler
app.use(function(req, res, next) {
  next(createError(404));
});

// error handler
app.use(function(err, req, res, next) {
  // set locals, only providing error in development
  res.locals.message = err.message;
  res.locals.error = req.app.get('env') === 'development' ? err : {};

  // render the error page
  res.status(err.status || 500);
  res.render('error');
});

module.exports = app;

