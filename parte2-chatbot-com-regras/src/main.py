import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import re
from difflib import get_close_matches
import nltk

nltk.download('punkt')

patterns = {
    "ir_para": r"(ir\s*para\s*|leve\s*me\s*para\s*|quero\s*ir\s*para\s*|me\s*leva\s*para\s*|vá\s*para\s*|quero\s*ir\s*para\s*|ir\s*para\s*o\s*)(\w+(\s*\w+)*)",
    "ir_ao": r"(ir\s*ao\s*|leve\s*me\s*ao\s*|quero\s*ir\s*ao\s*|me\s*leva\s*ao\s*|vá\s*para\s*ao\s*|quero\s*ir\s*ao\s*)(\w+(\s*\w+)*)",
}

def identificar_intencao(comando):
    comando = comando.lower()
    for intencao, padrao in patterns.items():
        match = re.search(padrao, comando)
        if match:
            local = match.group(2)
            return intencao, local
    return None, None

resposta_dict = "Entendido! O robô está indo para %s."

def acao_robot(intencao, local):
    if intencao and local:
        resposta = resposta_dict % local
        return resposta
    else:
        return "Comando não compreendido. Tente novamente."

def encontrar_intencao_mais_proxima(comando):
    comandos_disponiveis = ["ir_para", "ir_ao"]
    sugestao = get_close_matches(comando.lower(), comandos_disponiveis, n=1, cutoff=0.6)
    return sugestao[0] if sugestao else None

def corrigir_erro_local(local):
    locais_disponiveis = ["supermercado", "banheiro", "atelier", "biblioteca", "restaurante"]
    sugestao = get_close_matches(local, locais_disponiveis, n=1, cutoff=0.6)
    return sugestao[0] if sugestao else local

class ChatBotNode(Node):
    def __init__(self):
        super().__init__('chatbot_comando_robot')

        self.resposta_pub = self.create_publisher(String, 'resposta_usuario', 10)
        self.comando_pub = self.create_publisher(String, 'comando_usuario', 10)
        self.comando_sub = self.create_subscription(String, 'comando_usuario', self.comando_callback, 10)
        
    def comando_callback(self, msg):
        comando = msg.data
        self.get_logger().info(f"Comando recebido: {comando}")

        intencao, local = identificar_intencao(comando)

        if local:
            local_corrigido = corrigir_erro_local(local)
        else:
            local_corrigido = None

        resposta = acao_robot(intencao, local_corrigido)

        resposta_msg = String()
        resposta_msg.data = resposta

        self.resposta_pub.publish(resposta_msg)
        self.get_logger().info(f"Resposta enviada: {resposta_msg.data}")

def iniciar_conversa():
    print("\n")
    print("Chatbot - Comandos de Posicionamento do Robô.")
    print("Digite 'sair' para encerrar a conversa.")
    print("Use 'ir para {local}'.")
    print("\n")
    
    while True:
        comando_usuario = input("Você: ").strip()
        
        if comando_usuario.lower() == "sair":
            print("Chatbot: Adeus, tenha um ótimo dia!")
            break

        intencao, local = identificar_intencao(comando_usuario)
        if not intencao:
            intencao = encontrar_intencao_mais_proxima(comando_usuario)

        resposta = acao_robot(intencao, local)
        print(f"Chatbot: {resposta}")

def main(args=None):
    rclpy.init(args=args)
    iniciar_conversa()
    rclpy.shutdown()

if __name__ == '__main__':
    main()


