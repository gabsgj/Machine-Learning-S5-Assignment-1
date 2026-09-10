import os
import sys
import re
import subprocess
import shutil
import stat
import json
import urllib.request

# Master Class Roster
ROSTER = [
    {"reg": "TCR24CS001", "name": "Aakash P D"},
    {"reg": "TCR24CS002", "name": "Abhiram H"},
    {"reg": "TCR24CS003", "name": "Abhirami S"},
    {"reg": "TCR24CS004", "name": "Adal Seju"},
    {"reg": "TCR24CS005", "name": "Adithyan Vp"},
    {"reg": "TCR24CS006", "name": "Afeef Rahman U P"},
    {"reg": "TCR24CS007", "name": "A K Asif"},
    {"reg": "TCR24CS008", "name": "Alan Ali"},
    {"reg": "TCR24CS009", "name": "Alan T Robi"},
    {"reg": "TCR24CS010", "name": "Amaya Raveendran"},
    {"reg": "TCR24CS011", "name": "Amrutha K S"},
    {"reg": "TCR24CS012", "name": "Aneena S S"},
    {"reg": "TCR24CS013", "name": "Anna Tomy"},
    {"reg": "TCR24CS014", "name": "Archana K"},
    {"reg": "TCR24CS015", "name": "Archith Sunil"},
    {"reg": "TCR24CS016", "name": "Asni K"},
    {"reg": "TCR24CS018", "name": "Ayush Raj"},
    {"reg": "TCR24CS019", "name": "Bhagath P R"},
    {"reg": "TCR24CS020", "name": "Catherine Maria Benny"},
    {"reg": "TCR24CS021", "name": "Cilla Elsa Binoy"},
    {"reg": "TCR24CS022", "name": "David Chacko Binoy"},
    {"reg": "TCR24CS023", "name": "Devanandan JY"},
    {"reg": "TCR24CS024", "name": "Devika S"},
    {"reg": "TCR24CS025", "name": "Dilshath P K"},
    {"reg": "TCR24CS026", "name": "Emilin Suresh"},
    {"reg": "TCR24CS027", "name": "Farha T K"},
    {"reg": "TCR24CS028", "name": "Fasil Firose"},
    {"reg": "TCR24CS029", "name": "Gabriel James"},
    {"reg": "TCR24CS030", "name": "Gopika M S"},
    {"reg": "TCR24CS031", "name": "Govind C Menon"},
    {"reg": "TCR24CS032", "name": "Harikrishnan M"},
    {"reg": "TCR24CS033", "name": "HARINARAYANAN R"},
    {"reg": "TCR24CS034", "name": "Harshith M"},
    {"reg": "TCR24CS035", "name": "Jany sabarinath"},
    {"reg": "TCR24CS036", "name": "Jerin K Jaison"},
    {"reg": "TCR24CS037", "name": "Jerome Parekkattil"},
    {"reg": "TCR24CS038", "name": "Joseph Mathew"},
    {"reg": "TCR24CS040", "name": "Jovial James"},
    {"reg": "TCR24CS041", "name": "Karri Khohith"},
    {"reg": "TCR24CS042", "name": "Kasinathan A B"},
    {"reg": "TCR24CS043", "name": "Madhav Krishna T"},
    {"reg": "TCR24CS044", "name": "Manasa M"},
    {"reg": "TCR24CS045", "name": "Meera Nandita S"},
    {"reg": "TCR24CS046", "name": "Melita Mariam Mathew"},
    {"reg": "TCR24CS047", "name": "Nandana Mk"},
    {"reg": "TCR24CS048", "name": "Mridul Joy"},
    {"reg": "TCR24CS049", "name": "Muhammad Shanidh T P"},
    {"reg": "TCR24CS050", "name": "Nandana Sasikumar"},
    {"reg": "TCR24CS051", "name": "Nayana Shaji Mekkunnel"},
    {"reg": "TCR24CS052", "name": "Nevin Beno"},
    {"reg": "TCR24CS053", "name": "Niranjana Kanjoor"},
    {"reg": "TCR24CS054", "name": "NIRANJAN PP"},
    {"reg": "TCR24CS055", "name": "Pavithra S"},
    {"reg": "TCR24CS056", "name": "Rohan Shyam"},
    {"reg": "TCR24CS057", "name": "Rose Mary Ks"},
    {"reg": "TCR24CS058", "name": "Royce Pathayil Saji"},
    {"reg": "TCR24CS059", "name": "Sahil Shaji"},
    {"reg": "TCR24CS060", "name": "Shazia"},
    {"reg": "TCR24CS061", "name": "Shivas Seagal Ks"},
    {"reg": "TCR24CS062", "name": "Shobin Pn"},
    {"reg": "TCR24CS063", "name": "Sidharth V Jain"},
    {"reg": "TCR24CS064", "name": "Sivanandha K"},
    {"reg": "TCR24CS065", "name": "Sooraj K R"},
    {"reg": "TCR24CS066", "name": "Sreehari M Nambiar"},
    {"reg": "TCR24CS067", "name": "Sreelekshmi H"},
    {"reg": "TCR24CS068", "name": "Sreelekshmi Harikumar"},
    {"reg": "TCR24CS069", "name": "Surya T S"},
    {"reg": "TCR24CS070", "name": "V K Mohammed Shifaz"},
    {"reg": "TCR24CS071", "name": "VRINDHA P"},
    {"reg": "TCR24CS072", "name": "Dwaraka Dileep"},
    {"reg": "LTCR24CS073", "name": "KAVYA THILAKAN"},
    {"reg": "TCR24CS074", "name": "Muhammed Jiyad U"},
    {"reg": "TCR24CS075", "name": "Nabeel T"},
    {"reg": "TCR24CS076", "name": "Sabeel T"},
    {"reg": "KTE24CS077", "name": "Joseph John Paul"},
    {"reg": "TCR24CS077", "name": "Shima Shamsudheen"}
]

# Fast lookup by username / identifier
USERNAME_TO_REG = {
    'akasif7': 'TCR24CS007',
    'abhiramh7': 'TCR24CS002',
    'abbhiiraamii': 'TCR24CS003',
    'adal3396': 'TCR24CS004',
    'adithyanvp5': 'TCR24CS005',
    'afeef-official': 'TCR24CS006',
    'aakash-p-d': 'TCR24CS001',
    'alanali-byte': 'TCR24CS008',
    'alantrobi': 'TCR24CS009',
    'amayaraveendran': 'TCR24CS010',
    'amrutha1205': 'TCR24CS011',
    'aneena00': 'TCR24CS012',
    'anna-tomy': 'TCR24CS013',
    'archana774': 'TCR24CS014',
    'archithsunil': 'TCR24CS015',
    'asni016': 'TCR24CS016',
    'ayucpp': 'TCR24CS018',
    'bhagath-pr': 'TCR24CS019',
    'mariacatheriine': 'TCR24CS020',
    'cilla300': 'TCR24CS021',
    'david6969-eng': 'TCR24CS022',
    'devan4444': 'TCR24CS023',
    'dilshathpk': 'TCR24CS025',
    'dwarakadileep': 'TCR24CS072',
    'emilinsuresh': 'TCR24CS026',
    'fasil04': 'TCR24CS028',
    'farhatk101': 'TCR24CS027',
    'gabsgj': 'TCR24CS029',
    'gopika-m-s': 'TCR24CS030',
    'govindcmenon': 'TCR24CS031',
    'ha7-piixel': 'TCR24CS032',
    'hari2006-hash': 'TCR24CS033',
    'harshithm09': 'TCR24CS034',
    'jany104': 'TCR24CS035',
    'jerin-k-jaison': 'TCR24CS036',
    'jeromesumy': 'TCR24CS037',
    'josephjohnpaul': 'KTE24CS077',
    'joseph538-dev': 'TCR24CS038',
    'khohith': 'TCR24CS041',
    'kasi-ab': 'TCR24CS042',
    'elo0008': 'TCR24CS043',
    'manasam-madhavan': 'TCR24CS044',
    'meeralavender': 'TCR24CS045',
    'melita-28': 'TCR24CS046',
    'tetradox1412': 'TCR24CS048',
    'jiyad42': 'TCR24CS074',
    'nabeelt7034': 'TCR24CS075',
    'nandana2395': 'TCR24CS047',
    'nandgate1302': 'TCR24CS050',
    'nayanashaji': 'TCR24CS051',
    'nrj9595': 'TCR24CS054',
    'niranjanak18': 'TCR24CS053',
    'pavithra-s-hub': 'TCR24CS055',
    'rohan-shyam': 'TCR24CS056',
    'rosemaryks': 'TCR24CS057',
    'royce1415': 'TCR24CS058',
    'sabeel-t': 'TCR24CS076',
    '12anonymouz12': 'TCR24CS059',
    'shaziahabeeb': 'TCR24CS060',
    'shimuuh': 'TCR24CS077',
    'iamshobin': 'TCR24CS062',
    'sidharthvjain': 'TCR24CS063',
    'lumberrjackk': 'TCR24CS064',
    'pixelprogrammer4209': 'TCR24CS065',
    'sreelekshmi-h': 'TCR24CS067',
    'sreelekshmi-harikumar': 'TCR24CS068',
    'surya-t-s': 'TCR24CS069',
    'vrindhap': 'TCR24CS071',
    'devikashilu': 'TCR24CS024',
    'nevinbeno': 'TCR24CS052'
}

def remove_readonly(func, path, excinfo):
    try:
        os.chmod(path, stat.S_IWRITE)
        func(path)
    except Exception:
        pass

def normalize_git_url(url):
    url = url.split('?')[0].strip()
    url = re.sub(r'/blob/[^/]+/.*$', '', url)
    url = re.sub(r'/tree/[^/]+/?.*$', '', url)
    if url.endswith('.git'):
        url = url[:-4]
    url = url.rstrip('/')
    return url

def sort_roster_key(s):
    m = re.search(r'(\d{3})$', s['reg'])
    num = int(m.group(1)) if m else 999
    # Keep TCR before KTE if roll numbers match
    prefix = 0 if s['reg'].startswith('TCR') else (1 if s['reg'].startswith('LTCR') else 2)
    return (num, prefix)

def identify_student(raw_url, temp_dir=None):
    if 'colab.research.google.com' in raw_url:
        return 'TCR24CS061'

    for uname, reg in USERNAME_TO_REG.items():
        if f'/{uname}/' in raw_url.lower() or raw_url.lower().endswith(f'/{uname}'):
            return reg

    if temp_dir and os.path.exists(temp_dir):
        try:
            log_res = subprocess.run(['git', '-C', temp_dir, 'log', '-1', '--pretty=format:%an <%ae>'], capture_output=True, text=True)
            author_info = log_res.stdout.lower()
        except Exception:
            author_info = ''

        readme_content = ''
        for root, dirs, files in os.walk(temp_dir):
            for f in files:
                if f.lower().startswith('readme'):
                    try:
                        with open(os.path.join(root, f), 'r', errors='ignore') as rf:
                            readme_content += ' ' + rf.read()
                    except Exception:
                        pass

        combined_text = f'{author_info} {readme_content}'.lower()

        for s in ROSTER:
            if s['reg'].lower() in combined_text:
                return s['reg']

        for s in ROSTER:
            parts = [p.lower() for p in s['name'].split() if len(p) > 2]
            if parts and all(p in combined_text for p in parts):
                return s['reg']

    return None

def clean_dir(target_dir):
    for bad_item in ['.git', '__pycache__', '.DS_Store']:
        bad_path = os.path.join(target_dir, bad_item)
        if os.path.exists(bad_path):
            if os.path.isdir(bad_path):
                shutil.rmtree(bad_path, onerror=remove_readonly)
            else:
                try:
                    os.remove(bad_path)
                except Exception:
                    pass

def fetch_colab_notebook(url, target_dir, student_info):
    os.makedirs(target_dir, exist_ok=True)
    m = re.search(r'/drive/([a-zA-Z0-9_-]+)', url)
    if m:
        drive_id = m.group(1)
        export_url = f'https://drive.google.com/uc?export=download&id={drive_id}'
        dest_file = os.path.join(target_dir, 'winequality_red.ipynb')
        try:
            req = urllib.request.Request(export_url, headers={'User-Agent': 'Mozilla/5.0'})
            with urllib.request.urlopen(req) as resp, open(dest_file, 'wb') as out_f:
                out_f.write(resp.read())
            print(f'  -> Downloaded Colab notebook to {dest_file}')
        except Exception as e:
            print(f'  -> Failed to download notebook: {e}')

    readme_path = os.path.join(target_dir, 'README.md')
    with open(readme_path, 'w', encoding='utf-8') as f:
        f.write('# Machine Learning Assignment 1\n\n')
        f.write(f"**Student Name:** {student_info['name']}\n")
        f.write(f"**Registration Number:** {student_info['reg']}\n\n")
        f.write('## Google Colab Submission\n')
        f.write('This assignment was submitted via Google Colab notebook.\n\n')
        f.write(f'- [Open in Google Colab]({url})\n')
        f.write('- Local Notebook File: [`winequality_red.ipynb`](./winequality_red.ipynb)\n')

    with open(os.path.join(target_dir, '.repo_info'), 'w', encoding='utf-8') as f:
        f.write(url + '\n')

def generate_readme(base_dir, submissions_map):
    readme_path = os.path.join(base_dir, 'README.md')
    
    total_students = len(ROSTER)
    submitted_count = len([s for s in ROSTER if s['reg'] in submissions_map])
    missing_count = total_students - submitted_count
    submission_rate = (submitted_count / total_students) * 100

    content = []
    content.append('# Machine Learning - S4 Assignments')
    content.append('')
    content.append('## Assignment 1: Design of a Safe Semantic Planner in a Finite Cartesian State Space')
    content.append('**Course:** PCCST503 - Machine Learning | **Institution:** Government Engineering College, Thrissur (GECT)')
    content.append('')
    content.append('This repository is a consolidated collection of student submissions for **Machine Learning Assignment 1**. Each student has their own designated directory containing their source code, implementation, and assignment artifacts.')
    content.append('')
    content.append('---')
    content.append('')
    content.append('### 📊 Submission Overview')
    content.append('')
    content.append('| Metric | Count |')
    content.append('| :--- | :--- |')
    content.append(f'| **Total Students** | {total_students} |')
    content.append(f'| **Submissions Received** | {submitted_count} |')
    content.append(f'| **Missing / Pending** | {missing_count} |')
    content.append(f'| **Submission Rate** | {submission_rate:.1f}% |')
    content.append('')
    content.append('---')
    content.append('')
    content.append('### 📋 Student Submissions')
    content.append('')
    content.append('| # | Registration Number | Student Name | Status | Directory Link |')
    content.append('| :-: | :--- | :--- | :---: | :--- |')

    sorted_roster = sorted(ROSTER, key=sort_roster_key)

    for idx, student in enumerate(sorted_roster, 1):
        reg = student['reg']
        name = student['name']
        sub_info = submissions_map.get(reg)

        if sub_info:
            folder_name = sub_info['folder_name']
            clean_link = folder_name.replace(' ', '%20')
            status_badge = '✅ Submitted'
            if sub_info.get('is_colab'):
                dir_link = f'[{folder_name}](./{clean_link}) *(Google Colab)*'
            else:
                dir_link = f'[{folder_name}](./{clean_link})'
        else:
            status_badge = '❌ **Missing**'
            dir_link = '*Not Submitted*'

        content.append(f'| {idx} | `{reg}` | {name} | {status_badge} | {dir_link} |')

    content.append('')
    content.append('---')
    content.append('')
    content.append('### 🛠️ Repository Synchronization')
    content.append('To synchronize or fetch updated submissions, run:')
    content.append('```bash')
    content.append('python3 fetch_repos.py')
    content.append('```')
    content.append('')

    with open(readme_path, 'w', encoding='utf-8') as f:
        f.write('\n'.join(content))

    print(f'\nGenerated master README.md ({submitted_count} submitted, {missing_count} missing).')

def main():
    base_dir = os.path.dirname(os.path.abspath(__file__))
    txt_file = os.path.join(base_dir, 'ML Assignment 1.txt')
    
    if not os.path.exists(txt_file):
        print(f'Error: {txt_file} not found!')
        sys.exit(1)

    roster_by_reg = {s['reg']: s for s in ROSTER}

    entries = []
    with open(txt_file, 'r', encoding='utf-8', errors='ignore') as f:
        for line in f:
            line = line.strip()
            if not line or line.startswith('#'):
                continue
            parts = [p.strip() for p in line.split(',')]
            repo_url = parts[0]
            deploy_url = parts[1] if len(parts) > 1 else None
            entries.append({'url': repo_url, 'deploy': deploy_url})

    print(f'Loaded {len(entries)} submission URLs from ML Assignment 1.txt.')

    cache_clones_dir = '/tmp/ml_test_clones'
    submissions_map = {}

    for idx, entry in enumerate(entries, 1):
        raw_url = entry['url']
        deploy_url = entry['deploy']
        is_colab = 'colab.research.google.com' in raw_url

        print(f'\n[{idx}/{len(entries)}] Processing: {raw_url}')

        reg = identify_student(raw_url)
        if not reg:
            print(f'  [!] Warning: Could not identify student for {raw_url}')
            continue

        student = roster_by_reg[reg]
        folder_name = f"{student['reg']} - {student['name']}"
        folder_name = re.sub(r'[\\/*?:"<>|]', '', folder_name)
        target_path = os.path.join(base_dir, folder_name)

        submissions_map[reg] = {
            'student': student,
            'folder_name': folder_name,
            'url': raw_url,
            'is_colab': is_colab
        }

        info_file = os.path.join(target_path, '.repo_info')
        if os.path.exists(target_path) and os.path.exists(info_file):
            print(f"  -> Already exists at '{folder_name}'.")
            clean_dir(target_path)
            continue

        if is_colab:
            fetch_colab_notebook(raw_url, target_path, student)
            continue

        n_url = normalize_git_url(raw_url)
        cached_name = n_url.split('github.com/')[-1].replace('/', '__')
        cached_path = os.path.join(cache_clones_dir, cached_name)

        if os.path.exists(target_path):
            shutil.rmtree(target_path, onerror=remove_readonly)

        if os.path.exists(cached_path):
            print(f'  -> Copying from local cache: {cached_name}')
            shutil.copytree(cached_path, target_path)
        else:
            print(f'  -> Cloning fresh from GitHub: {n_url}')
            clone_url = n_url + '.git'
            res = subprocess.run(['git', 'clone', '--depth', '1', clone_url, target_path], capture_output=True, text=True)
            if res.returncode != 0:
                res = subprocess.run(['git', 'clone', '--depth', '1', n_url, target_path], capture_output=True, text=True)
                if res.returncode != 0:
                    print(f'  -> Clone failed: {res.stderr.strip()}')
                    continue

        clean_dir(target_path)

        with open(os.path.join(target_path, '.repo_info'), 'w', encoding='utf-8') as f:
            f.write(raw_url + '\n')

        if deploy_url:
            readme_target = os.path.join(target_path, 'README.md')
            try:
                content = ''
                if os.path.exists(readme_target):
                    with open(readme_target, 'r', errors='ignore') as rf:
                        content = rf.read()
                if deploy_url not in content:
                    with open(readme_target, 'a', encoding='utf-8') as rf:
                        rf.write(f'\n\n## Live Deployment\n[Live Link]({deploy_url})\n')
            except Exception as e:
                print(f'  -> Failed to append deploy link: {e}')

        print(f'  -> Saved to: {folder_name}')

    generate_readme(base_dir, submissions_map)

if __name__ == '__main__':
    main()
