import socket
import cv2
import pickle
import struct

def alici(host='localhost', port=9999):
    server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server_socket.bind((host, port))
    server_socket.listen(1)
    print(f"📥 Dinleniyor: {host}:{port}")

    conn, addr = server_socket.accept()
    print(f"✅ Bağlantı kuruldu: {addr}")

    data = b""
    payload_size = struct.calcsize("L")

    while True:
        while len(data) < payload_size:
            packet = conn.recv(4096)
            if not packet:
                return
            data += packet

        packed_msg_size = data[:payload_size]
        data = data[payload_size:]
        msg_size = struct.unpack("L", packed_msg_size)[0]

        while len(data) < msg_size:
            data += conn.recv(4096)

        frame_data = data[:msg_size]
        data = data[msg_size:]

        frame = pickle.loads(frame_data)
        frame = cv2.imdecode(frame, cv2.IMREAD_COLOR)

        cv2.imshow("Gelen Görüntü", frame)
        if cv2.waitKey(1) == 27:
            break

    conn.close()
    server_socket.close()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    alici()