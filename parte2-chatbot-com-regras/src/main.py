import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import re
from difflib import get_close_matches
import nltk
from nltk.tokenize import word_tokenize
import tkinter as tk
from tkinter import Frame, Text, Entry, Button

nltk.download('punkt')

patterns = {
    "ir_secretaria": r"(secretaria|escritório)",
    "ir_laboratorio": r"(laboratório|lab|laboratorio)",
    "ir_biblioteca": r"(biblioteca|livraria|biblia)"
}

def identificar_intencao(comando):
    comando = comando.lower()
    for intencao, padrao in patterns.items():
        if re.search(padrao, comando):
            return intencao
    return None

resposta_dict = {
    "ir_secretaria": "Entendido! O robô está indo para a secretaria.",
    "ir_laboratorio": "Entendido! O robô está indo para o laboratório.",
    "ir_biblioteca": "Entendido! O robô está indo para a biblioteca."
}

def acao_robot(intencao):
    if intencao:
        return resposta_dict.get(intencao, "Comando não reconhecido.")
    else:
        return "Comando não compreendido. Tente novamente."

def encontrar_intencao_mais_proxima(comando):
    comandos_disponiveis = ["ir_secretaria", "ir_laboratorio", "ir_biblioteca"]
    sugestao = get_close_matches(comando.lower(), comandos_disponiveis, n=1, cutoff=0.6)
    return sugestao[0] if sugestao else None

class ChatBotNode(Node):
    def __init__(self):
        super().__init__('chatbot_comando_robot')
        
        self.resposta_pub = self.create_publisher(String, 'resposta_usuario', 10)
        
        self.comando_pub = self.create_publisher(String, 'comando_usuario', 10)  # ADICIONADO AQUI
        
        self.comando_sub = self.create_subscription(
            String,
            'comando_usuario',
            self.comando_callback,
            10
        )
        
    def comando_callback(self, msg):
        comando = msg.data
        self.get_logger().info(f"Comando recebido: {comando}")
        
        intencao = encontrar_intencao_mais_proxima(comando)
        
        resposta = acao_robot(intencao)
        
        resposta_msg = String()
        resposta_msg.data = resposta
        
        self.resposta_pub.publish(resposta_msg)
        self.get_logger().info(f"Resposta enviada: {resposta_msg.data}")

class ChatBotGUI:
    def __init__(self, root):
        self.context = ""
        self.root = root
        self.root.title("Chatbot - Comando Robot")
        self.root.geometry("700x750")

        self.conversation_history = Text(root, bg='white')
        self.conversation_history.config(state='disabled')
        self.conversation_history.pack()

        self.user_input_frame = Frame(root)
        self.user_input_frame.pack()

        self.user_input_field = Entry(self.user_input_frame)
        self.user_input_field.bind("<Return>", self.send_message)
        self.user_input_field.pack(side='left')

        self.send_button = Button(self.user_input_frame, text="Enviar", command=self.send_message)
        self.send_button.pack(side='left')

        self.clear_history_button = Button(self.user_input_frame, text="Limpar Histórico", command=self.clear_history)
        self.clear_history_button.pack(side='left')

        self.clear_input_button = Button(self.user_input_frame, text="Limpar Entrada", command=self.clear_input)
        self.clear_input_button.pack(side='left')

        self.chatbot = ChatBotNode()
        self.chatbot_thread = None

    def send_message(self, event=None):
        user_input = self.user_input_field.get()
        self.conversation_history.config(state='normal')
        self.conversation_history.insert('insert', f"You: {user_input}\n")
        self.conversation_history.config(state='disabled')
        self.conversation_history.see('end')

        comando_msg = String()
        comando_msg.data = user_input
        self.chatbot.comando_pub.publish(comando_msg)

        if user_input.lower() == "quit":
            bot_response = "Adeus, tenha um ótimo dia!"
        else:
            bot_response = self.get_bot_response(user_input)

        self.conversation_history.config(state='normal')
        self.conversation_history.insert('insert', f"Chatbot: {bot_response}\n")
        self.conversation_history.config(state='disabled')
        self.conversation_history.see('end')
        self.user_input_field.delete(0, 'end')

    def clear_history(self):
        self.conversation_history.config(state='normal')
        self.conversation_history.delete(1.0, 'end')
        self.conversation_history.config(state='disabled')

    def clear_input(self):
        self.user_input_field.delete(0, 'end')

    def get_bot_response(self, user_input):
        if "biblioteca" in user_input:
            return "O robô está indo para a biblioteca."
        elif "secretaria" in user_input:
            return "O robô está indo para a secretaria."
        elif "laboratório" in user_input:
            return "O robô está indo para o laboratório."
        else:
            return "Desculpe, não entendi o que você disse. Pode tentar novamente?"

def main(args=None):
    rclpy.init(args=args)
    
    root = tk.Tk()
    gui = ChatBotGUI(root)

    root.mainloop()
    
    rclpy.shutdown()

if __name__ == '__main__':
    main()
