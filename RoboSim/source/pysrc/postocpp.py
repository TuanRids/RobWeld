"""================================================================================================================================================================
    Testing when using .py directly
    START
"""
def give_to_cpp():
    def testtk():
        import tkinter as tk
        # Tạo một cửa sổ chính
        root = tk.Tk()
        # Đặt tiêu đề cho cửa sổ
        root.title("Cửa sổ Tkinter đơn giản")
        # Đặt kích thước cho cửa sổ (chiều rộng x chiều cao)
        root.geometry("400x300")
        # Tạo một nhãn để hiển thị văn bản
        label = tk.Label(root, text="Xin chào! Đây là một cửa\n sổ Tkinter đơn giản.", font=("Helvetica", 16))
        label.pack(pady=20)
        # Tạo một nút để thoát khi ứng dụng
        exit_button = tk.Button(root, text="Thoát", command=root.quit)
        exit_button.pack(pady=10)
        # Chạy vòng lặp sự kiện chính của ứng dụng
        root.mainloop()


    testtk()
    # initialize the value
    endvl = [
        [460, -200, 200, 180, -90, 0],
        [660, -100, 200, 180, -90, 0],
        [460, 0, 200, 180, -90, 0],
        [660, 100, 200, 180, -90, 0],
        [460, 200, 200, 180, -90, 0]
    ]
    # convert to float
    endvl_float = [[float(val) for val in sublist] for sublist in endvl]
    # return 2D array
    return endvl_float
"""
    Testing when using .py directly
    END
"""

"""================================================================================================================================================================
    Testing when using .exe
    START

def give_to_cpp():
    # initialize the value
    endvl = [
        [460, -200, 200, 180, -90, 0],
        [660, -100, 200, 180, -90, 0],
        [460, 0, 200, 180, -90, 0],
        [660, 100, 200, 180, -90, 0],
        [460, 200, 200, 180, -90, 0]
    ]
    # convert to float
    endvl_float = [[float(val) for val in sublist] for sublist in endvl]
    # print 2D array
    for sublist in endvl_float:
        print(' '.join(map(str, sublist)))

if __name__ == "__main__":
    give_to_cpp()


    Testing when using .exe
    END
"""