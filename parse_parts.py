import csv
import re
import math
from fractions import Fraction

def fraction_to_decimal(fraction_str):
    """Convert fraction strings to decimal values"""
    if not fraction_str:
        return 0.0
    
    # Clean the string
    fraction_str = fraction_str.strip().replace('"', '')
    
    # Handle whole numbers
    if fraction_str.isdigit():
        return float(fraction_str)
    
    # Handle mixed numbers like "1 1/4" or "1-1/4"
    mixed_pattern = r'(\d+)[-\s]+(\d+)/(\d+)'
    mixed_match = re.match(mixed_pattern, fraction_str)
    if mixed_match:
        whole = int(mixed_match.group(1))
        numerator = int(mixed_match.group(2))
        denominator = int(mixed_match.group(3))
        return whole + (numerator / denominator)
    
    # Handle simple fractions like "3/4"
    fraction_pattern = r'(\d+)/(\d+)'
    fraction_match = re.match(fraction_pattern, fraction_str)
    if fraction_match:
        numerator = int(fraction_match.group(1))
        denominator = int(fraction_match.group(2))
        return numerator / denominator
    
    # Handle decimal numbers
    try:
        return float(fraction_str)
    except ValueError:
        return 0.0

def parse_diameter(description):
    """Extract diameter from description"""
    # Look for diameter at the beginning of description
    # Patterns: "3/4", "1 1/4", "1-1/4", "1""
    patterns = [
        r'^(\d+\s+\d+/\d+)',  # "1 1/4"
        r'^(\d+-\d+/\d+)',    # "1-1/4" 
        r'^(\d+/\d+)',        # "3/4"
        r'^(\d+")',           # "1""
        r'^(\d+\.?\d*)',      # "1.5" or "1"
    ]
    
    for pattern in patterns:
        match = re.search(pattern, description)
        if match:
            diameter_str = match.group(1).replace('"', '')
            return fraction_to_decimal(diameter_str)
    
    return None

def parse_length(description):
    """Extract length from description after 'X'"""
    # Look for length after "X" pattern
    # Patterns: "X 6 1/4", "X 3-1/2", "X 12"
    patterns = [
        r'X\s+(\d+\s+\d+/\d+)',  # "X 6 1/4"
        r'X\s+(\d+-\d+/\d+)',    # "X 3-1/2"
        r'X\s+(\d+/\d+)',        # "X 3/4"
        r'X\s+(\d+\.?\d*)',      # "X 12" or "X 12.5"
    ]
    
    for pattern in patterns:
        match = re.search(pattern, description)
        if match:
            length_str = match.group(1)
            return fraction_to_decimal(length_str)
    
    return None

def parse_head_type(description):
    """Extract head type from description"""
    if '12PT' in description:
        return '12PT'
    elif 'HVY HEX' in description:
        return '6PT'
    else:
        return 'UNKNOWN'

def clean_quantity(qty_str):
    """Clean quantity string and convert to integer"""
    if isinstance(qty_str, str):
        # Remove commas and convert to int
        return int(qty_str.replace(',', ''))
    return int(qty_str)

def calculate_weight(diameter_inches, length_inches):
    """Calculate weight of cylindrical 4140 steel part in kg"""
    # 4140 steel density: 7.85 g/cm³ = 0.284 lb/in³
    # Convert to kg/in³: 0.284 lb/in³ * 0.453592 kg/lb = 0.1288 kg/in³
    steel_density = 0.1288  # kg/in³
    
    # Convert diameter to radius
    radius_inches = diameter_inches / 2.0
    
    # Calculate volume of cylinder: π * r² * h
    volume_cubic_inches = math.pi * (radius_inches ** 2) * length_inches
    
    # Calculate weight: volume * density
    weight_kg = volume_cubic_inches * steel_density
    
    return round(weight_kg, 3)

def classify_payload(weight_kg, bin_size_kg=0.5):
    """Classify part into payload number based on weight bins"""
    # Payload 1: 0-0.5kg, Payload 2: 0.5-1.0kg, etc.
    # Using 0.5kg bin size for good granularity across 0.099-1.897kg range
    payload_number = math.ceil(weight_kg / bin_size_kg)
    return max(1, payload_number)  # Ensure minimum payload number is 1

def main():
    """Main function to parse the parts list"""
    input_file = '2024 parts list.csv'
    output_file = 'cleanparts.csv'
    
    parsed_parts = []
    
    # Read the input CSV
    with open(input_file, 'r', encoding='utf-8-sig') as file:
        reader = csv.DictReader(file)
        
        for row in reader:
            description = row['Desc']
            qty = row['QTY']
            
            # Skip empty rows
            if not description.strip():
                continue
            
            # Parse each field
            diameter = parse_diameter(description)
            length = parse_length(description)
            head_type = parse_head_type(description)
            quantity = clean_quantity(qty)
            
            # Only add if we successfully parsed diameter and length
            if diameter is not None and length is not None:
                weight = calculate_weight(diameter, length)
                payload_number = classify_payload(weight)
                parsed_parts.append({
                    'part_diameter': diameter,
                    'part_length': length,
                    'head_type': head_type,
                    'quantity': quantity,
                    'weight_kg': weight,
                    'payload_number': payload_number
                })
    
    # Write the output CSV
    with open(output_file, 'w', newline='') as file:
        fieldnames = ['part_diameter', 'part_length', 'head_type', 'quantity', 'weight_kg', 'payload_number']
        writer = csv.DictWriter(file, fieldnames=fieldnames)
        
        writer.writeheader()
        for part in parsed_parts:
            writer.writerow(part)
    
    print(f"Parsed {len(parsed_parts)} parts and saved to {output_file}")
    
    # Print first few rows for verification
    print("\nFirst 5 rows:")
    for i, part in enumerate(parsed_parts[:5]):
        print(f"{i+1}: {part}")

if __name__ == "__main__":
    main()