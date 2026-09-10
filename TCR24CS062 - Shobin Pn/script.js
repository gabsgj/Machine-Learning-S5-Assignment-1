const runButton = document.getElementById("runPlanner");
const resultArea = document.getElementById("result");

runButton.addEventListener("click", async () => {

    resultArea.innerHTML = "Planning...";

    try {

        const response = await fetch(
            "http://localhost:8080/plan"
        );

        if (!response.ok) {
            throw new Error("Server returned an error");
        }

        const data = await response.json();

        if (!data.success) {

            resultArea.innerHTML = `
                <div class="error">
                    No safe path found.
                </div>
            `;

            return;
        }

        const path =
            data.path.join(" → ");

        const safety =
            (data.safety * 100).toFixed(0);

        resultArea.innerHTML = `
            <div class="success">

                <h2>Safe Path Found</h2>

                <p>
                    <strong>Path:</strong>
                    ${path}
                </p>

                <p>
                    <strong>Total Cost:</strong>
                    ${data.cost}
                </p>

                <p>
                    <strong>Safety:</strong>
                    ${safety}%
                </p>

            </div>
        `;

    } catch (error) {

        console.error(error);

        resultArea.innerHTML = `
            <div class="error">
                Could not connect to the planner server.
                <br><br>
                Make sure the C++ server is running.
            </div>
        `;
    }
});