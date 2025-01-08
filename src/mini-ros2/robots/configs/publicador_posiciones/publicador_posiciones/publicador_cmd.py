import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped
import time

# Variable global para almacenar la posición actual del robot
current_pose = None

def amcl_pose_callback(msg):
    global current_pose
    current_pose = msg.pose.pose
    print("Posición actual del robot actualizada: ({}, {})".format(current_pose.position.x, current_pose.position.y))

def send_goal(node, position_x, position_y, orientation_z=0.0, orientation_w=1.0, times_list=None):
    global current_pose

    # Crear un publicador para el tópico /goal_pose
    goal_publisher = node.create_publisher(PoseStamped, '/goal_pose', 10)

    # Crear un suscriptor para el tópico de posición estimada del robot
    node.create_subscription(PoseWithCovarianceStamped, '/amcl_pose', amcl_pose_callback, 10)

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
            distance_to_goal = ((current_pose.position.x - position_x) ** 2 +
                                (current_pose.position.y - position_y) ** 2) ** 0.5
            node.get_logger().info("Distancia al objetivo: {:.2f}".format(distance_to_goal))
            if distance_to_goal < 0.5:  # Tolerancia de 0.5 metros
                # Registrar el tiempo de finalización
                end_time = time.time()
                elapsed_time = end_time - start_time
                node.get_logger().info("Tiempo para alcanzar el objetivo: {:.2f} segundos".format(elapsed_time))

                # Guardar el tiempo y la meta en la lista de tiempos
                if times_list is not None:
                    times_list.append((position_x, position_y, elapsed_time))
                break
        else:
            node.get_logger().warn("Esperando actualización de la posición del robot...")
        rclpy.spin_once(node, timeout_sec=1.0)

def main(args=None):
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

        # Lista para almacenar los tiempos registrados
        times = []

        # Enviar las metas una por una en el orden especificado
        for position_x, position_y in goals:
            send_goal(node, position_x, position_y, 0.0, 1.0, times)
            time.sleep(5.0)  # Esperar un breve tiempo después de enviar cada meta

        # Imprimir los tiempos registrados
        node.get_logger().info("Resultados:")
        for position_x, position_y, elapsed_time in times:
            node.get_logger().info("Meta: ({}, {}), Tiempo: {:.2f} segundos".format(position_x, position_y, elapsed_time))

        # Guardar los resultados en un archivo
        with open('resultados_tiempos.txt', 'w') as f:
            for position_x, position_y, elapsed_time in times:
                f.write("Meta: ({}, {}), Tiempo: {:.2f} segundos\n".format(position_x, position_y, elapsed_time))

    except KeyboardInterrupt:
        pass

    # Limpiar y cerrar el nodo
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

