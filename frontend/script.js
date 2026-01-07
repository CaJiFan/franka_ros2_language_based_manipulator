document.addEventListener('DOMContentLoaded', () => {
    // State
    let tools = [];
    let sequence = [];
    let handedness = 'right';

    // DOM Elements
    const toolListEl = document.getElementById('tool-list');
    const newToolInput = document.getElementById('new-tool-name');
    const addToolBtn = document.getElementById('add-tool-btn');
    
    const toolSelector = document.getElementById('tool-selector');
    const addToSeqBtn = document.getElementById('add-to-sequence-btn');
    const sequenceListEl = document.getElementById('sequence-list');
    const startSeqBtn = document.getElementById('start-sequence-btn');
    
    const exportBtn = document.getElementById('export-btn');
    const importFile = document.getElementById('import-file');
    const configOutput = document.getElementById('config-output');

    const handLeftBtn = document.getElementById('hand-left-btn');
    const handRightBtn = document.getElementById('hand-right-btn');

    // --- Initialization ---
    renderToolList();
    updateToolSelector();
    renderSequence();
    updateHandednessUI(); // Set initial UI state

    // --- Handedness Logic ---
    handLeftBtn.addEventListener('click', () => {
        handedness = 'left';
        updateHandednessUI();
        updateConfigPreview(); // Update JSON preview immediately
    });

    handRightBtn.addEventListener('click', () => {
        handedness = 'right';
        updateHandednessUI();
        updateConfigPreview(); // Update JSON preview immediately
    });

    function updateHandednessUI() {
        if (handedness === 'left') {
            // Set Left as Active (Primary style)
            handLeftBtn.classList.remove('outline');
            handLeftBtn.classList.add('primary');
            
            // Set Right as Inactive (Outline style)
            handRightBtn.classList.add('outline');
            handRightBtn.classList.remove('primary');
        } else {
            // Set Right as Active
            handRightBtn.classList.remove('outline');
            handRightBtn.classList.add('primary');
            
            // Set Left as Inactive
            handLeftBtn.classList.add('outline');
            handLeftBtn.classList.remove('primary');
        }
    }

    // --- Tool Management ---
    addToolBtn.addEventListener('click', () => {
        const name = newToolInput.value.trim();
        if (name && !tools.includes(name)) {
            tools.push(name);
            newToolInput.value = '';
            renderToolList();
            updateToolSelector();
        } else if (tools.includes(name)) {
            alert('Tool already exists!');
        }
    });

    function renderToolList() {
        toolListEl.innerHTML = '';
        tools.forEach(tool => {
            const li = document.createElement('li');
            li.className = 'list-item';
            li.innerHTML = `
                <span>${tool}</span>
                <button class="btn danger" onclick="removeTool('${tool}')"><i class="fas fa-trash"></i></button>
            `;
            toolListEl.appendChild(li);
        });
    }

    window.removeTool = (toolName) => {
        if (confirm(`Delete "${toolName}"?`)) {
            tools = tools.filter(t => t !== toolName);
            renderToolList();
            updateToolSelector();
        }
    };

    function updateToolSelector() {
        toolSelector.innerHTML = '<option value="" disabled selected>Select a tool to add...</option>';
        tools.forEach(tool => {
            const option = document.createElement('option');
            option.value = tool;
            option.textContent = tool;
            toolSelector.appendChild(option);
        });
    }

    // --- Sequence Management ---
    addToSeqBtn.addEventListener('click', () => {
        const selectedTool = toolSelector.value;
        if (selectedTool) {
            // Default start time: last item's time + 10, or 0 if empty
            const lastTime = sequence.length > 0 ? sequence[sequence.length - 1].startTime : -10;
            
            sequence.push({
                id: Date.now(),
                tool: selectedTool,
                startTime: lastTime + 10
            });
            renderSequence();
        }
    });

    function renderSequence() {
        sequenceListEl.innerHTML = '';
        
        if (sequence.length === 0) {
            sequenceListEl.innerHTML = '<div class="empty-state">No steps in sequence yet. Add a tool to start.</div>';
            return;
        }

        sequence.forEach((step, index) => {
            const div = document.createElement('div');
            div.className = 'sequence-item';
            div.draggable = true; // Enable dragging
            div.dataset.index = index; // Store index for drag logic
            
            div.innerHTML = `
                <span class="seq-number"><i class="fas fa-grip-vertical" style="color:#ccc; margin-right:5px;"></i> #${index + 1}</span>
                <div class="seq-tool">${step.tool}</div>
                <div class="seq-time">
                    <label>Ready at (sec):</label>
                    <input type="number" value="${step.startTime}" min="0" onchange="updateStartTime(${step.id}, this.value)">
                </div>
                <div class="seq-controls">
                    <button class="seq-btn delete" onclick="removeStep(${step.id})" title="Remove"><i class="fas fa-times"></i></button>
                </div>
            `;
            
            // Add Drag Events
            addDragEvents(div);

            sequenceListEl.appendChild(div);
        });
        
        updateConfigPreview();
    }

    // --- Drag and Drop Logic ---
    let draggedItemIndex = null;

    function addDragEvents(item) {
        item.addEventListener('dragstart', (e) => {
            draggedItemIndex = parseInt(item.dataset.index);
            item.classList.add('dragging');
            e.dataTransfer.effectAllowed = 'move';
        });

        item.addEventListener('dragend', () => {
            item.classList.remove('dragging');
            draggedItemIndex = null;
            
            // Remove any drag-over styles from all items
            document.querySelectorAll('.sequence-item').forEach(el => {
                el.style.borderTop = '';
                el.style.borderBottom = '';
            });
        });

        item.addEventListener('dragover', (e) => {
            e.preventDefault(); // Necessary to allow dropping
            e.dataTransfer.dropEffect = 'move';
        });
        
        item.addEventListener('drop', (e) => {
            e.preventDefault();
            const targetIndex = parseInt(item.dataset.index);
            
            if (draggedItemIndex !== null && draggedItemIndex !== targetIndex) {
                // Move item in array
                const itemToMove = sequence[draggedItemIndex];
                sequence.splice(draggedItemIndex, 1); // Remove from old pos
                sequence.splice(targetIndex, 0, itemToMove); // Insert at new pos
                
                renderSequence();
            }
        });
    }

    // Allow dropping on the container (for empty spaces or end of list)
    sequenceListEl.addEventListener('dragover', (e) => {
        e.preventDefault();
    });


    window.updateStartTime = (id, value) => {
        const step = sequence.find(s => s.id === id);
        if (step) step.startTime = parseInt(value) || 0;
        updateConfigPreview();
    };

    window.removeStep = (id) => {
        sequence = sequence.filter(s => s.id !== id);
        renderSequence();
    };

    // --- Execution ---
    startSeqBtn.addEventListener('click', () => {
        if (sequence.length === 0) {
            alert('Please define a sequence first.');
            return;
        }
        
        // Sort sequence by time before sending? 
        // Usually sequences are executed in order of the list, but if time is absolute, 
        // we might want to sort them. For now, let's keep list order as the source of truth.
        // Or, we can sort them by startTime automatically?
        // Let's sort them for the "Start" action to be safe.
        const sortedSequence = [...sequence].sort((a, b) => a.startTime - b.startTime);

        console.log('Starting Sequence:', sortedSequence);
        alert(`Dum-E Sequence Started!\n\nExecuting ${sortedSequence.length} steps.\nCheck console for details.`);
    });

    // --- Audio Recording ---
    const recordBtn = document.getElementById('record-btn');
    const stopRecordBtn = document.getElementById('stop-record-btn');
    const recordingStatus = document.getElementById('recording-status');
    const audioPlayback = document.getElementById('audio-playback');

    let mediaRecorder;
    let audioChunks = [];
    let audioBlob = null;

    if (recordBtn && stopRecordBtn) {
        recordBtn.addEventListener('click', async () => {
            try {
                const stream = await navigator.mediaDevices.getUserMedia({ audio: true });
                mediaRecorder = new MediaRecorder(stream);
                audioChunks = [];

                mediaRecorder.addEventListener('dataavailable', event => {
                    audioChunks.push(event.data);
                });

                mediaRecorder.addEventListener('stop', () => {
                    // Create blob from chunks
                    audioBlob = new Blob(audioChunks, { type: 'audio/webm' });
                    const audioUrl = URL.createObjectURL(audioBlob);
                    
                    // Setup playback
                    audioPlayback.src = audioUrl;
                    audioPlayback.style.display = 'block';
                    recordingStatus.textContent = 'Recording finished. Ready to play.';
                    
                    // Stop all tracks to release microphone
                    stream.getTracks().forEach(track => track.stop());
                    
                    console.log("Audio recording stored in 'audioBlob' variable.");
                });

                mediaRecorder.start();
                recordBtn.disabled = true;
                stopRecordBtn.disabled = false;
                recordingStatus.textContent = 'Recording...';
                audioPlayback.style.display = 'none';
            } catch (err) {
                console.error('Error accessing microphone:', err);
                recordingStatus.textContent = 'Error accessing microphone: ' + err.message;
            }
        });

        stopRecordBtn.addEventListener('click', () => {
            if (mediaRecorder && mediaRecorder.state !== 'inactive') {
                mediaRecorder.stop();
                recordBtn.disabled = false;
                stopRecordBtn.disabled = true;
            }
        });
    }

    // --- File I/O ---
    function updateConfigPreview() {
        const config = {
            project: "Dum-E",
            handedness: handedness,
            tools: tools,
            sequence: sequence.map(s => ({ tool: s.tool, startTime: s.startTime }))
        };
        configOutput.value = JSON.stringify(config, null, 2);
    }

    exportBtn.addEventListener('click', () => {
        const config = configOutput.value;
        const blob = new Blob([config], { type: 'application/json' });
        const url = URL.createObjectURL(blob);
        const a = document.createElement('a');
        a.href = url;
        a.download = 'dum-e-sequence.json';
        document.body.appendChild(a);
        a.click();
        document.body.removeChild(a);
        URL.revokeObjectURL(url);
    });

    importFile.addEventListener('change', (e) => {
        const file = e.target.files[0];
        if (!file) return;

        const reader = new FileReader();
        reader.onload = (event) => {
            try {
                const data = JSON.parse(event.target.result);
                
                if (data.tools && Array.isArray(data.tools)) {
                    tools = data.tools;
                }
                if (data.sequence && Array.isArray(data.sequence)) {
                    sequence = data.sequence.map(s => ({
                        id: Date.now() + Math.random(),
                        tool: s.tool,
                        startTime: s.startTime !== undefined ? s.startTime : (s.duration || 0) // Fallback for old files
                    }));
                }
                if (data.handedness) {
                    handedness = data.handedness;
                    updateHandednessUI();
                }

                renderToolList();
                updateToolSelector();
                renderSequence();
                alert('Configuration loaded successfully!');
            } catch (err) {
                alert('Error parsing JSON file. Please check the format.');
                console.error(err);
            }
        };
        reader.readAsText(file);
    });
});
