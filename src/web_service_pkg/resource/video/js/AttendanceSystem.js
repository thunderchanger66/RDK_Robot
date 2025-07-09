// 添加打卡记录到表格
function addAttendanceRecord(record) {
    const tableBody = document.getElementById('recordsBody');
    
    // 创建新行
    const row = document.createElement('tr');
    
    // 添加日期单元格
    const dateCell = document.createElement('td');
    dateCell.textContent = record.date;
    row.appendChild(dateCell);
    
    // 添加时间单元格
    const timeCell = document.createElement('td');
    timeCell.textContent = record.time;
    row.appendChild(timeCell);
    
    // 添加姓名单元格
    const nameCell = document.createElement('td');
    nameCell.textContent = record.name;
    row.appendChild(nameCell);

    // 添加工具单元格
    const toolCell = document.createElement('td');
    toolCell.textContent = record.tool || "未指定";
    row.appendChild(toolCell);
    
    // 将新记录插入表格顶部
    tableBody.insertBefore(row, tableBody.firstChild);
}

// 搜索打卡记录
function searchRecords() {
    const query = document.getElementById('searchInput').value.trim().toLowerCase();
    if (!query) {
        alert('请输入姓名或工具名称进行查询！');
        return;
    }

    const tableBody = document.getElementById('recordsBody');
    const rows = tableBody.getElementsByTagName('tr');
    let found = false;

    for (let row of rows) {
        const name = row.cells[2].textContent.toLowerCase(); // 姓名在第3列
        const tool = row.cells[3].textContent.toLowerCase(); // 工具在第4列
        
        if (name.includes(query) || tool.includes(query)) {
            row.style.display = '';
            found = true;
        } else {
            row.style.display = 'none';
        }
    }

    if (!found) {
        alert('未找到相关记录！');
    }
}

// 新增手动工具选择功能
function manualToolSelection(toolName) {
    currentTool = toolName;
    console.log('手动选择工具:', toolName);
    alert(`已选择工具: ${toolName}`);
}