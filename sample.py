from selenium import webdriver
from selenium.webdriver.common.keys import Keys

# Initialize the Chrome WebDriver
driver = webdriver.Chrome()

# Open the desired webpage
driver.get("https://www.google.com")

# Find the search input element by its name attribute
search_box = driver.find_element_by_name("q")

# Enter the search query
search_box.send_keys("OpenAI")

# Submit the search query
search_box.send_keys(Keys.RETURN)

# Wait for a moment to see the results (for demonstration purposes)
input("Press any key to exit...")

# Close the browser
driver.quit()
