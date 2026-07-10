import os
import re
import sys

def collect_unique_chinese_chars(directory):
    """收集目录中所有文件的中文字符，去重后合并为一行"""
    # 中文字符的Unicode范围（基本汉字和扩展汉字）
    chinese_pattern = re.compile(r'[\u4e00-\u9fff\u3400-\u4dbf\U00020000-\U0002a6df]')
    
    # 使用集合来存储唯一的中文字符
    unique_chars = set()
    processed_files = 0
    skipped_files = 0
    
    # 支持的文本文件扩展名
    text_extensions = {'.cpp', '.h', '.c', '.hpp', '.txt', '.py', '.java', '.js', '.html', '.css', '.xml'}
    
    print(f"正在扫描文件夹: {directory}")
    
    # 遍历目录中的所有文件
    for root, dirs, files in os.walk(directory):
        for file in files:
            file_path = os.path.join(root, file)
            file_ext = os.path.splitext(file_path)[1].lower()
            
            # 只处理文本文件
            if file_ext not in text_extensions:
                continue
                
            try:
                # 尝试多种编码打开文件
                encodings = ['utf-8', 'gbk', 'gb2312', 'latin-1']
                content = None
                
                for encoding in encodings:
                    try:
                        with open(file_path, 'r', encoding=encoding) as f:
                            content = f.read()
                        break
                    except UnicodeDecodeError:
                        continue
                
                if content is None:
                    print(f"警告: 无法读取文件 {file} (编码问题)")
                    skipped_files += 1
                    continue
                
                # 移除//注释
                lines = content.split('\n')
                cleaned_content = []
                for line in lines:
                    # 查找//注释的位置
                    comment_index = line.find('//')
                    if comment_index >= 0:
                        # 保留注释前的部分
                        cleaned_content.append(line[:comment_index])
                    else:
                        cleaned_content.append(line)
                
                # 重新组合内容
                cleaned_content = '\n'.join(cleaned_content)
                
                # 收集所有中文字符
                matches = chinese_pattern.findall(cleaned_content)
                for char in matches:
                    unique_chars.add(char)
                
                processed_files += 1
                
            except Exception as e:
                print(f"警告: 处理文件 {file} 时出错: {str(e)}")
                skipped_files += 1
    
    # 将去重后的中文字符合并为一行
    unique_chars_line = ''.join(sorted(unique_chars))
    
    print(f"\n处理完成!")
    print(f"已处理文件: {processed_files}")
    print(f"跳过文件: {skipped_files}")
    print(f"唯一中文字符数量: {len(unique_chars)}")
    print(f"所有唯一中文字符: {unique_chars_line}")

if __name__ == "__main__":
    # 获取命令行参数
    directory = sys.argv[1] if len(sys.argv) > 1 else "."
    collect_unique_chinese_chars(directory)