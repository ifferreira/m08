import tkinter as tk
import ollama
import PyPDF2
from sentence_transformers import SentenceTransformer
from scipy.spatial.distance import cosine
import numpy as np

embedding_model = SentenceTransformer('all-MiniLM-L6-v2')  # Você pode usar outro modelo aqui

def read_pdf(pdf_path):
    with open(pdf_path, 'rb') as file:
        reader = PyPDF2.PdfReader(file)
        text = ""
        for page in reader.pages:
            text += page.extract_text()
    return text

def create_embeddings(text):
    sentences = text.split('\n')
    embeddings = embedding_model.encode(sentences)
    return sentences, embeddings

def find_relevant_context(user_input, sentences, embeddings):
    question_embedding = embedding_model.encode([user_input])[0]
    similarities = [1 - cosine(question_embedding, emb) for emb in embeddings]
    most_relevant_idx = np.argmax(similarities)
    return sentences[most_relevant_idx]

def generate_response(user_input, context):
    input_text = f"{context}\n\nPergunta: {user_input}\nResposta:"
    response = ollama.chat(model="llama3", messages=[{"role": "user", "content": input_text}])
    return response['message']['content']

def send_message():
    user_input = entry.get()
    
    if user_input.strip() == "":
        return
    
    relevant_context = find_relevant_context(user_input, pdf_sentences, pdf_embeddings)
    
    response = generate_response(user_input, relevant_context)
    
    chat_box.config(state=tk.NORMAL)
    chat_box.insert(tk.END, f"Você: {user_input}\n")
    chat_box.insert(tk.END, f"Chatbot: {response}\n\n")
    chat_box.config(state=tk.DISABLED)
    
    entry.delete(0, tk.END)

pdf_path = 'normas_seguranca.pdf'  # Caminho do seu PDF
pdf_text = read_pdf(pdf_path)
pdf_sentences, pdf_embeddings = create_embeddings(pdf_text)

root = tk.Tk()
root.title("Chatbot de Segurança Industrial")

chat_box = tk.Text(root, height=20, width=50, state=tk.DISABLED)
chat_box.pack(padx=10, pady=10)

entry = tk.Entry(root, width=50)
entry.pack(padx=10, pady=10)

send_button = tk.Button(root, text="Enviar", command=send_message)
send_button.pack(padx=10, pady=10)

root.mainloop()


