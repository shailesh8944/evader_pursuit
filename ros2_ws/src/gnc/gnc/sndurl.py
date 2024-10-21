import subprocess
import re
import time

# Define the ngrok command
command = "ngrok http 8500"

# Open ngrok in a new terminal window
subprocess.Popen(['gnome-terminal', '--', 'bash', '-c', command])

# Give ngrok some time to start up
time.sleep(2)

# Now we can check the ngrok logs to find the public URL
try:
    # Check the ngrok logs for the public URL
    log_command = "curl -s http://localhost:4040/api/tunnels"
    result = subprocess.run(log_command, shell=True, check=True, stdout=subprocess.PIPE, text=True)

    # Print the entire output for reference
    print("ngrok API Output:")
    print(result.stdout)

    # Use a regex to find the public URL in the output
    url_pattern = re.compile(r'https?://[a-zA-Z0-9-]+\.ngrok\.io')
    urls = url_pattern.findall(result.stdout)

    if urls:
        print("Public URL:")
        print(urls[0])  # Print the first found URL
    else:
        print("No public URL found in the output.")

except subprocess.CalledProcessError as e:
    print("Error occurred while accessing ngrok API:")
    print(e.stderr)
