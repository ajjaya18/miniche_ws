import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped, Twist
import time

# Variables globales
current_pose = None
current_twist = None  # Velocidades actuales

# Listas para almacenar las velocidades lineales, angulares y sus respectivos tiempos
time_stamps = []
linear_velocities = []
angular_velocities = []

def amcl_pose_callback(msg):
    global current_pose
    current_pose = msg.pose.pose

def twist_callback(msg):
    global current_twist, time_stamps, linear_velocities, angular_velocities

    current_twist = msg

    # Guardar las velocidades lineales, angulares y los tiempos en las listas
    time_stamps.append(time.time())
    linear_velocities.append(msg.linear.x)
    angular_velocities.append(msg.angular.z)

def calculate_covariance(data1, data2):
    """Calcula la covarianza entre dos listas de datos."""
    if len(data1) != len(data2) or len(data1) == 0:
        return 0.0  # No se puede calcular la covarianza si las listas están vacías o de tamaño distinto

    mean1 = sum(data1) / len(data1)
    mean2 = sum(data2) / len(data2)

    covariance = sum((x - mean1) * (y - mean2) for x, y in zip(data1, data2)) / len(data1)
    return covariance

def send_goal(node, position_x, position_y, orientation_z=0.0, orientation_w=1.0):
    global current_pose, current_twist

    # Crear un publicador para el tópico /goal_pose
    goal_publisher = node.create_publisher(PoseStamped, '/goal_pose', 10)

    # Crear suscriptores para la posición y las velocidades
    node.create_subscription(PoseWithCovarianceStamped, '/amcl_pose', amcl_pose_callback, 10)
    node.create_subscription(Twist, '/cmd_vel', twist_callback, 10)

    # Esperar a que los publicadores y suscriptores estén listos
    rclpy.spin_once(node, timeout_sec=1.0)

    # Crear un mensaje de tipo PoseStamped para la posición y orientación especificadas
    goal_msg = PoseStamped()
    goal_msg.header.stamp = node.get_clock().now().to_msg()
    goal_msg.header.frame_id = 'map'
    goal_msg.pose.position.x = position_x
    goal_msg.pose.position.y = position_y
    goal_msg.pose.position.z = 0.0
    goal_msg.pose.orientation.x = 0.0
    goal_msg.pose.orientation.y = 0.0
    goal_msg.pose.orientation.z = orientation_z
    goal_msg.pose.orientation.w = orientation_w

    # Registrar el tiempo de inicio
    start_time = time.time()

    # Publicar el mensaje en el tópico /goal_pose
    goal_publisher.publish(goal_msg)

    # Mostrar un mensaje de confirmación
    node.get_logger().info("Se ha enviado el objetivo: ({}, {})".format(position_x, position_y))

    # Esperar a que el robot alcance la posición deseada
    while rclpy.ok():
        if current_pose is not None:
            # Calcular la distancia al objetivo
            distance_to_goal = ((current_pose.position.x - position_x) ** 2 +
                                (current_pose.position.y - position_y) ** 2) ** 0.5
            node.get_logger().info("Distancia al objetivo: {}".format(distance_to_goal))

            # Verificar si se alcanzó el objetivo
            if distance_to_goal < 0.5:  # Tolerancia de 0.5 metros
                end_time = time.time()
                elapsed_time = end_time - start_time
                node.get_logger().info("Tiempo para alcanzar el objetivo: {:.2f} segundos".format(elapsed_time))
                break
        else:
            node.get_logger().warn("Esperando actualización de la posición del robot...")
        rclpy.spin_once(node, timeout_sec=0.1)

def main(args=None):
    global time_stamps, linear_velocities, angular_velocities

    rclpy.init(args=args)
    node = rclpy.create_node('send_goal_node')

    try:
        # Lista de metas (coordenadas) en orden de envío
        goals = [
            (-2.5924670696258545, 1.6289286613464355),
            (0.43468546867370605, 0.17821013927459717),
            (2.6550140380859375, -0.9098283052444458),
            (3.024038076400757, 1.997838020324707)
        ]

        # Enviar las metas una por una en el orden especificado
        for position_x, position_y in goals:
            send_goal(node, position_x, position_y, 0.0, 1.0)
            time.sleep(5.0)  # Esperar un breve tiempo después de enviar cada meta

        # Calcular diferencias de tiempo
        time_deltas = [t - time_stamps[0] for t in time_stamps]

        # Calcular la covarianza de las velocidades lineales y angulares con respecto al tiempo
        linear_time_covariance = calculate_covariance(linear_velocities, time_deltas)
        angular_time_covariance = calculate_covariance(angular_velocities, time_deltas)

        node.get_logger().info("Covarianza de velocidad lineal con el tiempo: {:.6f}".format(linear_time_covariance))
        node.get_logger().info("Covarianza de velocidad angular con el tiempo: {:.6f}".format(angular_time_covariance))

    except KeyboardInterrupt:
        pass

    # Limpiar y cerrar el nodo
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()