from flask import Flask, render_template, request, jsonify
import PythonProject as PP

app = Flask(__name__)
graph = PP.main.creator()
@app.route("/")
def index():
 return render_template('form.html')

@app.route('/graphs')
def graphs():
 return jsonify(graph)

if __name__  == '__main__':
 app.run(debug=True)