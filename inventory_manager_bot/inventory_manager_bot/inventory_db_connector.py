import psycopg2

class InventoryDBConnector():
    def __init__(self):
        pass

    def connect_to_db(self,db,host,user,password,port):
        self.connector = psycopg2.connect(database=db,
                                        host=host,
                                        user=user,
                                        password=password,
                                        port=port)
        
        self.cursor = self.connector.cursor()
        

    def insert_to_db(self,product_no,item_name,brand_name,weight,quantity,aisle_text,checkin,checkout):
        query = "INSERT INTO inventory" + \
                    "(product_no,product_name,brand,weight,quantity,aisle,checkin,checkout_date)" + \
                    "VALUES ('{}','{}','{}',{},{},'{}',TO_TIMESTAMP('{}','YYYY-MM-DD HH24:MI:SS'),TO_DATE('{}','YYYY-MM-DD'))" \
                        .format(product_no,item_name,brand_name,weight,quantity,aisle_text,checkin,checkout)
        print("Performing current query...")
        self.cursor.execute(query)
        self.connector.commit()

    def check_checkout_is_null(self,product_no):
        query = "SELECT to_char(actual_checkout,'YYYY-MM-DD HH24:MI:SS') FROM inventory" + \
                    " WHERE product_no = '{}'".format(product_no)
        print("Performing current query...")
        self.cursor.execute(query)
        return self.cursor.fetchone()

    def check_if_product_no_exists(self,product_no):
        query = "SELECT * FROM inventory" + \
                    " WHERE product_no = '{}'".format(product_no)
        print("Performing current query...")
        self.cursor.execute(query)
        return self.cursor.fetchone()

    def get_expected_checkout_date_from_db(self,product_no):
        query = "SELECT to_char(checkout_date, 'Mon DD, YYYY') FROM inventory" + \
                " WHERE product_no = '{}'".format(product_no)
        self.cursor.execute(query)
        return self.cursor.fetchone()

    def submit_checkout_to_db(self,checkout,product_no):
        query = "UPDATE inventory" + \
                " SET actual_checkout = TO_TIMESTAMP('{}','YYYY-MM-DD HH24:MI:SS')".format(checkout) + \
                " WHERE product_no = '{}'".format(product_no)
        print("Performing current query...")
        self.cursor.execute(query)
        self.connector.commit()

    def get_all_records_from_word(self,word):
        query = "SELECT * FROM inventory" + \
                " WHERE product_name LIKE '%{}%'".format(word)
        print("Performing current query...")
        self.cursor.execute(query)
        return self.cursor.fetchall()
    
    def get_aisle_from_product_no(self,number):
        query = "SELECT aisle FROM inventory" + \
                " WHERE product_no = '{}'".format(number)
        print("Performing current query...")
        self.cursor.execute(query)
        return self.cursor.fetchone()


    def close_connection(self):
        print("Closing connection to database...")
        self.connector.close()

if __name__ == "__main__":
    conn = InventoryDBConnector()
    try:
        conn.connect_to_db()
        print("Database connection established!")
        conn.query_db("SELECT * FROM inventory")
    except Exception as e:
        print("Could not connect to database!")
        print(e)