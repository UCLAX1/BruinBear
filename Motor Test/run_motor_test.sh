#!/bin/bash

# Start the Java server (assuming MyJavaApp.java contains the server code)
echo "Starting Java Gateway Server..."
javac -cp "/home/akhilesh/venv/share/py4j/py4j0.10.9.9.jar" MyJavaApp.java

# Run the Java server in the background and capture the process ID (PID)
java -cp ".:/home/akhilesh/venv/share/py4j/py4j0.10.9.9.jar" MyJavaApp &
JAVA_PID=$!

# Wait for the Java server to initialize (you can adjust this based on your needs)
sleep 3

# Run the Python script
echo "Running Python script..."
python3 /home/akhilesh/BruinBear/Motor\ Test/client.py

# Kill the Java Gateway Server process after the Python script finishes
echo "Killing Java Gateway Server process..."
kill -9 $JAVA_PID
