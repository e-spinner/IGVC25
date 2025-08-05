import rclpy
from rclpy.node import Node

import pkg_resources
from flask import Flask, jsonify, render_template
from flask_socketio import SocketIO, emit
from threading import Thread
import os
import time

from sir.msg import Map

class Dashboard(Node):

  def __init__(self) -> None:
    super().__init__("display")

    # Create Subscriptions
    # ----------------------------------------------------------------
    self.map_update_count = 0
    self.map_sub = self.create_subscription(
      Map,
      '/map',
      self.map_callback,
      10
    )


    # Create Flask App
    # ----------------------------------------------------------------
    try:
      # Get the installed package directory
      template_dir = pkg_resources.resource_filename('sir_dashboard', 'templates')
      static_dir = pkg_resources.resource_filename('sir_dashboard', 'static')

      self.app = Flask(__name__,
                      template_folder=template_dir,
                      static_folder=static_dir)

      self.get_logger().info(f"Templates directory: {template_dir}")
      self.get_logger().info(f"Static directory: {static_dir}")

    except Exception as e:
        self.get_logger().warn(f"Could not find package templates, using relative path: {e}")
        # Fallback to relative path
        self.app = Flask(__name__)

    # initialize socketio
    self.app.config['SECRET_KEY'] = 'secret'
    self.socketio = SocketIO(self.app)

    self.setup_routes()
    self.setup_events()
    self.get_logger().info("Dashboard Node initialized")


  # Subscription Callbacks
  # ----------------------------------------------------------------
  def map_callback(self, msg: Map) -> None:
    try:
      self.map_update_count += 1

      print(msg.data)

      self.socketio.emit('map_update', '')
      self.get_logger().info(f"Broadcasting map update #{self.map_update_count} to all clients")

    except Exception as e:
      self.get_logger().error(f"Error processing map data: {e}")
      # Send error to clients
      error_data = {
          'timestamp': time.time(),
          'status': 'error',
          'error_message': str(e)
      }
      self.socketio.emit('map_error', error_data)


  # Create Flask Routes
  # ----------------------------------------------------------------
  def setup_routes(self) -> None:

    @self.app.route('/')
    def index() -> str:
      return render_template('dashboard.html')

  def run_flask(self) -> None:
    self.socketio.run(self.app, port=5000, debug=False)

  # Create SocketIO Events
  # ----------------------------------------------------------------
  def setup_events(self) -> None:

    @self.socketio.on('connect')
    def connect() -> None:
      print('client connected')
      emit('status', {'msg': 'Connected to Dashboard'})

    self.socketio.on('diconnect')
    def disconnect() -> None:
      print('client disconnected')

def main(args=None) -> None:
  rclpy.init(args=args)

  dashboard = Dashboard()
  flask_thread = Thread(target=dashboard.run_flask, daemon=True)
  flask_thread.start()
  dashboard.get_logger().info("Dashboard running at http://localhost:5000")

  try:
    rclpy.spin(dashboard)

  except KeyboardInterrupt:
    pass

  finally:
    rclpy.shutdown()

if __name__ == "__main__":
  main()