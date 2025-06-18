import os
import subprocess

token = os.getenv("token")
# Your local repo path
repo_path = "./"

# Remote URL to push to
remote_url = "https://"+token+"@github.com/mdecycu/cad2025.git"  # or use SSH URL

# Run git push to the remote URL
result = subprocess.run(
    ["git", "push", remote_url],
    cwd=repo_path,
    capture_output=True,
    text=True
)

# Output result
print("STDOUT:", result.stdout)
print("STDERR:", result.stderr)

# Check if successful
if result.returncode == 0:
    print("✅ Push successful.")
else:
    print("❌ Push failed.")
