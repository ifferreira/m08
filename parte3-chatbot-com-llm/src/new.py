import tkinter as tk
import ollama
import PyPDF2
from sentence_transformers import SentenceTransformer
from scipy.spatial.distance import cosine
import numpy as np
import nltk
from nltk.tokenize import sent_tokenize

nltk.download('punkt', download_dir='./nltk_data')
nltk.download('punkt_tab')
nltk.data.path.append('./nltk_data')

embedding_model = SentenceTransformer('all-MiniLM-L6-v2')

def read_pdf(pdf_path):
    with open(pdf_path, 'rb') as file:
        reader = PyPDF2.PdfReader(file)
        text = ""
        for page in reader.pages:
            text += page.extract_text()
    return text

def create_embeddings(text):
    sentences = sent_tokenize(text)
    embeddings = embedding_model.encode(sentences)
    return sentences, embeddings

def find_relevant_context(user_input, sentences, embeddings, window=2):
    question_embedding = embedding_model.encode([user_input])[0]
    similarities = [1 - cosine(question_embedding, emb) for emb in embeddings]
    most_relevant_idx = np.argmax(similarities)
    
    start = max(0, most_relevant_idx - window)
    end = min(len(sentences), most_relevant_idx + window + 1)
    return " ".join(sentences[start:end])

def generate_response(user_input, context):
    input_text = (
        f"Texto do currículo:\n{context}\n\n"
        f"Pergunta: {user_input}\n"
        f"Resposta:"
    )
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

pdf_path = 'C:/Users/Inteli/Repositories/m08/parte3-chatbot-com-llm/src/normas_seguranca.pdf'
pdf_text = read_pdf(pdf_path)
pdf_sentences, pdf_embeddings = create_embeddings(pdf_text)

root = tk.Tk()
root.title("Chatbot")

chat_box = tk.Text(root, height=20, width=50, state=tk.DISABLED)
chat_box.pack(padx=10, pady=10)

entry = tk.Entry(root, width=50)
entry.pack(padx=10, pady=10)

send_button = tk.Button(root, text="Enviar", command=send_message)
send_button.pack(padx=10, pady=10)

root.mainloop()



